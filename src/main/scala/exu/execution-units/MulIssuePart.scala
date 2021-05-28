package boom.exu


import scala.collection.mutable.ArrayBuffer
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.{BP, PipelinedMultiplier}
import freechips.rocketchip.tile.{RoCCCoreIO, XLen}
import freechips.rocketchip.tile
import FUConstants._
import boom.common._
import boom.ifu.GetPCFromFtqIO
import boom.util.{BoomCoreStringPrefix, BranchKillableQueue, GetNewBrMask, ImmGen, IsKilledByBranch}


class MulIssuePartInBundle(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle
{
  val valid = Input(Bool())
  val ready = Output(Bool())

  val dec = Input(new MulDecodeIO)
  val zeroDetect = Input(new MulZeroDetectIO)
  val rs1_data = Input(UInt(dataWidth.W))
  val rs2_data = Input(UInt(dataWidth.W))

  val uop = Input(new MicroOp)
}

class MulIssuePart(numReadPort: Int, dataWidth: Int, numIssueSlots: Int)(implicit p: Parameters) extends BoomModule{
  val io = IO(new Bundle{
    val in = Vec(numReadPort, new MulIssuePartInBundle(dataWidth))
    val packet = Vec(numReadPort, Valid(new Packet))

    val brupdate = Input(new BrUpdateInfo())
    val kill = Input(Bool())
  })

  val s_invalid :: s_valid_1 :: Nil = Enum(2)

  //-------------------------------------------------------------
  // Set up the dispatch uops

  val dis_contain = Array.fill(numReadPort) {Wire(new SlotContain)}
  for (w <- 0 until numReadPort) {
    dis_contain(w).uop := io.in(w).uop
    dis_contain(w).uop.muliw_state := s_valid_1
    dis_contain(w).tag := io.in(w).zeroDetect.tag
    dis_contain(w).pattern := io.in(w).zeroDetect.pattern
    dis_contain(w).req := io.in(w).zeroDetect.request
    dis_contain(w).cmd_hi := io.in(w).dec.cmdHi
    dis_contain(w).cmd_half := io.in(w).dec.cmdHalf
    dis_contain(w).data(0) := Cat(0.U(1.W), io.in(w).rs1_data(dataWidth/2 - 1, 0)).asSInt
    dis_contain(w).data(1) := Cat(io.in(w).dec.lhsSigned && io.in(w).rs1_data(dataWidth-1), io.in(w).rs1_data(dataWidth-1, dataWidth/2)).asSInt
    dis_contain(w).data(2) := Cat(0.U(1.W), io.in(w).rs2_data(dataWidth/2 - 1, 0)).asSInt
    dis_contain(w).data(3) := Cat(io.in(w).dec.rhsSigned && io.in(w).rs2_data(dataWidth-1), io.in(w).rs2_data(dataWidth-1, dataWidth/2)).asSInt
  }

  //-------------------------------------------------------------
  // Issue Table

  val slots = for (i <- 0 until numIssueSlots) yield { val slot = Module(new MulIssueSlot); slot }
  val issue_slots = VecInit(slots.map(_.io))

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).brupdate         := io.brupdate
    issue_slots(i).kill             := io.kill
  }

  val count = PopCount(slots.map(_.io.valid))
  dontTouch(count)

  //-------------------------------------------------------------

//  assert(PopCount(issue_slots.map(s => s.grant)) <= numReadPort.U, "[issue] window giving out too many grants.")


  //-------------------------------------------------------------
  // Figure out how much to shift entries by

  val maxShift = numReadPort
  val vacants = issue_slots.map(s => !(s.valid)) ++ io.in.map(_.valid).map(!_.asBool)
  val shamts_oh = Array.fill(numIssueSlots+numReadPort) {Wire(UInt(width=maxShift.W))}
  // track how many to shift up this entry by by counting previous vacant spots
  def SaturatingCounterOH(count_oh:UInt, inc: Bool, max: Int): UInt = {
    val next = Wire(UInt(width=max.W))
    next := count_oh
    when (count_oh === 0.U && inc) {
      next := 1.U
    } .elsewhen (!count_oh(max-1) && inc) {
      next := (count_oh << 1.U)
    }
    next
  }
  shamts_oh(0) := 0.U
  for (i <- 1 until numIssueSlots + numReadPort) {
    shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), maxShift)
  }

  //-------------------------------------------------------------

  // which entries' uops will still be next cycle? (not being issued and vacated)
  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
    (0 until numReadPort).map(i => io.in(i).valid)

  val contains = issue_slots.map(s=>s.out) ++ dis_contain.map(s=>s)
  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in.valid := false.B    // default is not move to here
    issue_slots(i).in.bits  := contains(i+1)
    for (j <- 1 to maxShift by 1) {
      when (shamts_oh(i+j) === (1 << (j-1)).U) {    // is this slot data move to here
        issue_slots(i).in.valid := will_be_valid(i+j)
        issue_slots(i).in.bits  := contains(i+j)
      }
    }
    issue_slots(i).clear        := shamts_oh(i) =/= 0.U   // has slot data move to here
  }

  //-------------------------------------------------------------
  // Dispatch/Entry Logic
  // did we find a spot to slide the new dispatched uops into?

  val will_be_available = (0 until numIssueSlots).map(i =>
    (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in.valid))
  val num_available = PopCount(will_be_available)
  for (w <- 0 until numReadPort) {
    io.in(w).ready := RegNext(num_available > w.U + 2 * numReadPort.U)
  }

  //-------------------------------------------------------------
  // Issue Select Logic

  // set default
  for (w <- 0 until numReadPort) {
    io.packet(w).valid := false.B
    io.packet(w).bits.uop := NullMicroOp
    io.packet(w).bits.weight := 0.U
    io.packet(w).bits.tag := 0.U
    io.packet(w).bits.rs1_x := 0.S
    io.packet(w).bits.rs2_x := 0.S
    io.packet(w).bits.pattern := 0.U
  }

  val requests = issue_slots.map(s => s.reqs)
  val port_issued = Array.fill(numReadPort){Bool()}    // CAUTION: Arrays are mutable
  for (w <- 0 until numReadPort) {
    port_issued(w) = false.B
  }

  for (i <- 0 until numIssueSlots) {    // allow issue queue have internal vacant
    for (j <- 0 until 4) {
      issue_slots(i).grant(j) := 0.U
      var uop_issued = false.B // CAUTION: var, uop_issued = a | uop_issued, first uop_issued and second uop_issued are not the same

      for (w <- 0 until numReadPort) {
        when(requests(i)(j) && !uop_issued && !port_issued(w)) {
          issue_slots(i).grant(j) := 1.U
          io.packet(w).valid := true.B
          io.packet(w).bits := issue_slots(i).packet(j)
        }
        val was_port_issued_yet = port_issued(w)
        port_issued(w) = (requests(i)(j) && !uop_issued) | port_issued(w)
        uop_issued = (requests(i)(j) && !was_port_issued_yet) | uop_issued
      }
    }
  }



}
