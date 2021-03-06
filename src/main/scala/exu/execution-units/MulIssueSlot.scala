package boom.exu


import scala.collection.mutable.ArrayBuffer
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.BP
import freechips.rocketchip.tile.{RoCCCoreIO, XLen}
import freechips.rocketchip.tile
import FUConstants._
import boom.common._
import boom.ifu.GetPCFromFtqIO
import boom.util.{BoomCoreStringPrefix, BranchKillableQueue, GetNewBrMask, ImmGen, IsKilledByBranch}


class SlotContain()(implicit  p: Parameters) extends BoomBundle
{
  val uop = new MicroOp() // if valid, this WILL overwrite an entry!
  val tag = UInt(2.W)
  val data = Vec(4, SInt(33.W))
  val req = UInt(4.W)
  val pattern = UInt(2.W)
  val cmd_hi = Bool()
  val cmd_half = Bool()
}

class Packet()(implicit p: Parameters) extends BoomBundle
{
  val uop = new MicroOp()
  val tag = UInt(2.W)
  val rs1_x = SInt(33.W)
  val rs2_x = SInt(33.W)
  val weight = UInt(2.W)
  val pattern = UInt(2.W)
  val cmd_hi = Bool()
  val cmd_half = Bool()
}

class MulIssueSlotIO()(implicit p: Parameters) extends BoomBundle
{
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool()) // TODO code review, do we need this signal so explicitely?
  val request       = Output(Bool())  // TODO: the same to valid, can remove it
  val reqs          = Output(UInt(4.W))
  val grant         = Input(Vec(4, UInt(1.W)))

  val brupdate      = Input(new BrUpdateInfo())
  val kill          = Input(Bool()) // pipeline flush
  val clear         = Input(Bool()) // entry being moved elsewhere (not mutually exclusive with grant)

  val in            = Flipped(Valid(new SlotContain())) // if valid, this WILL overwrite an entry!
  val out           = Output(new SlotContain()) // the updated slot uop; will be shifted upwards in a collasping queue.
  val packet        = Output(Vec(4, (new Packet()))) // the current Slot's uop. Sent down the pipeline when issued.
}

class MulIssueSlot()(implicit p: Parameters) extends BoomModule {
  val io = IO(new MulIssueSlotIO)

  val s_invalid :: s_valid_1 :: Nil = Enum(2)
  val state = RegInit(s_invalid)

  // slot invalid?
  // slot is valid, holding 1 uop
  def is_invalid = state === s_invalid
  def is_valid = state =/= s_invalid

  val next_state      = Wire(UInt()) // the next state of this slot (which might then get moved to a new slot)
  val next_uopc       = Wire(UInt()) // the next uopc of this slot (which might then get moved to a new slot)

  val slot_contain = RegInit({
    val slot_init = Wire(new SlotContain)
    slot_init.tag := 0.U(2.W)
    slot_init.data := VecInit(Seq.fill(4)(0.S(33.W)))
    slot_init.req := 0.U(4.W)
    slot_init.pattern := 0.U(2.W)
    slot_init.uop := NullMicroOp
    slot_init.cmd_half := false.B
    slot_init.cmd_hi := false.B
    slot_init
  })

//  val next_contain = Mux(io.in.valid, io.in.bits, slot_contain)

  //-----------------------------------------------------------------------------
  // next slot state computation
  // compute the next state for THIS entry slot (in a collasping queue, the
  // current uop may get moved elsewhere, and a new uop can enter

  when (io.kill) {
    state := s_invalid
  } .elsewhen (io.in.valid) {
    state := io.in.bits.uop.muliw_state  // TODO: need to initialized in issue unit  // can create new one because the width changed
  } .elsewhen (io.clear) {
    state := s_invalid
    slot_contain.req := 0.U(4.W)
  } .otherwise {
    state := next_state
  }

  //-----------------------------------------------------------------------------
  // "update" state
  // compute the next state for the micro-op in this slot. This micro-op may
  // be moved elsewhere, so the "next_state" travels with it.

  // defaults
  next_state := state
  next_uopc := slot_contain.uop.uopc

  val cnt_req = PopCount(slot_contain.req)
  val cnt_grant = PopCount(io.grant.asUInt)
  val next_req = Wire(Vec(4, UInt(1.W)))
  next_req := slot_contain.req.asBools
  val slot_packets = Wire(Vec(4, (new Packet)))

  def connectPacket(slot_packet: Packet, i: Int, j: Int, weight:Int) = {
    slot_packet.pattern := slot_contain.pattern
    slot_packet.uop := slot_contain.uop
    slot_packet.rs1_x := slot_contain.data(i)
    slot_packet.rs2_x := slot_contain.data(j)
    slot_packet.weight := weight.U
    slot_packet.tag := slot_contain.tag
    slot_packet.cmd_half := slot_contain.cmd_half
    slot_packet.cmd_hi := slot_contain.cmd_hi
    require(i != j)
  }
  connectPacket(slot_packets(0), 0, 2, 0)
  connectPacket(slot_packets(1), 0, 3, 1)
  connectPacket(slot_packets(2), 1, 2, 2)
  connectPacket(slot_packets(3), 1, 3, 3)

  when (io.kill) {
    next_state := s_invalid
  } .elsewhen(cnt_grant =/= 0.U && (state === s_valid_1)){
    when(cnt_req === cnt_grant){
      next_state := s_invalid
      next_req := 0.U(4.W).asBools
    }.elsewhen(cnt_req > cnt_grant) {
      next_state := s_valid_1
      next_req := slot_contain.req.asBools
      for (i <- 0 until 4) {
        when(io.grant(i) === 1.U){
          next_req(i) := false.B
        }
      }
    }
    slot_contain.req := next_req.asUInt
  }

  when (io.in.valid) {
    slot_contain := io.in.bits
    assert (is_invalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
  }

  // Handle branch misspeculations
  val next_br_mask = GetNewBrMask(io.brupdate, slot_contain.uop)

  // was this micro-op killed by a branch? if yes, we can't let it be valid if
  // we compact it into an other entry
  when (IsKilledByBranch(io.brupdate, slot_contain.uop)) {
    next_state := s_invalid
  }

  when (!io.in.valid) {
    slot_contain.uop.br_mask := next_br_mask
  }

  //-------------------------------------------------------------
  // Request Logic
  io.request := is_valid && !io.kill
  io.reqs := slot_contain.req

  when (state === s_valid_1) {
    io.request := !io.kill
  }.otherwise {
    io.request := false.B
  }

  //assign outputs
  io.valid := is_valid
  io.packet := slot_packets


  // micro-op will vacate due to grant.
  val may_vacate = cnt_grant === cnt_req && state === s_valid_1
  io.will_be_valid := is_valid && !may_vacate

  io.out                := slot_contain
  io.out.uop.muliw_state   := next_state
  io.out.uop.uopc       := next_uopc

  io.out.uop.br_mask    := next_br_mask

}


//      when(cnt_req === 2.U) {
//        // try to issue
//        when(slot_contain.req(0) === 1.U) {
//          next_req = slot_contain.req
//          next_req(0) := 0.U
//          slot_packets(0).valid := slot_contain.req(0)
//        }.elsewhen(slot_contain.req(1) === 1.U) {
//          next_req = slot_contain.req
//          next_req(1) := 0.U
//          slot_packets(1).valid := slot_contain.req(1)
//        }.elsewhen(slot_contain.req(2) === 1.U) {
//          next_req = slot_contain.req
//          next_req(2) := 0.U
//          slot_packets(2).valid := slot_contain.req(2)
//        }.elsewhen(slot_contain.req(3) === 1.U) {
//          next_req = slot_contain.req
//          next_req(3) := 0.U
//          slot_packets(3).valid := slot_contain.req(3)
//        }
//      }