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

class MultiPortExeUnitIOr(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle {
  val fu_types = Output(Bits(FUC_SZ.W))
  val req = Flipped(new DecoupledIO(new FuncUnitReq(dataWidth)))
  val brupdate = Input(new BrUpdateInfo())
}

class MultiPortExeUnitIOw(val numBypassStages: Int, val dataWidth: Int)(implicit p: Parameters) extends BoomBundle {
  val iresp = new DecoupledIO(new ExeUnitResp(dataWidth))
  val bypass = Output(Vec(numBypassStages, Valid(new ExeUnitResp(dataWidth))))
}

class MultiPortExeUnit(
                  val numBypassStages: Int,
                  val dataWidth: Int,
                  val numReadPort: Int,
                  val numWritePort: Int
                )(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle() {
    val rp = Vec(numReadPort, new MultiPortExeUnitIOr(dataWidth))
    val wp = Vec(numWritePort, new MultiPortExeUnitIOw(numBypassStages, dataWidth))
  })
  
  for (i <- 0 until numWritePort){
    io.wp(i).iresp.bits.fflags.valid := false.B
    io.wp(i).iresp.bits.predicated := false.B
    assert(io.wp(i).iresp.ready)
  }

  for (i <- 0 until numReadPort){
    io.rp(i).fu_types := FU_MUL
  }
  
  def supportedFuncUnits = {
    new SupportedFuncUnits(
      alu = false,
      jmp = false,
      mem = false,
      muld = true,
      fpu = false,
      csr = false,
      fdiv = false,
      ifpu = false)
  }
}

// First use pipelined iMul to verify can add to BOOM
class MulExeUnit(
                  numBypassStages: Int = 0,
                  numReadPort: Int = 4,
                  numWritePort: Int = 6
                )(implicit p: Parameters)
  extends MultiPortExeUnit(
    numBypassStages,
    p(tile.XLen) + 1,
    numReadPort,
    numWritePort
  )
{
  val numStages = 3
  require(numReadPort == numWritePort)
  for (i <- 0 until numReadPort) {
    // Pipelined functional unit is always ready.
    io.rp(i).req.ready := true.B

    if (numStages > 0) {
      val r_valids = RegInit(VecInit(Seq.fill(numStages) {
        false.B
      }))
      val r_uops = Reg(Vec(numStages, new MicroOp()))

      // handle incoming request
      r_valids(0) := io.rp(i).req.valid && !IsKilledByBranch(io.rp(i).brupdate, io.rp(i).req.bits.uop) && !io.rp(i).req.bits.kill
      r_uops(0) := io.rp(i).req.bits.uop
      r_uops(0).br_mask := GetNewBrMask(io.rp(i).brupdate, io.rp(i).req.bits.uop)

      // handle middle of the pipeline
      for (i <- 1 until numStages) {
        r_valids(i) := r_valids(i - 1) && !IsKilledByBranch(io.rp(i).brupdate, r_uops(i - 1)) && !io.rp(i).req.bits.kill
        r_uops(i) := r_uops(i - 1)
        r_uops(i).br_mask := GetNewBrMask(io.rp(i).brupdate, r_uops(i - 1))
      }

      // handle outgoing (branch could still kill it)
      // consumer must also check for pipeline flushes (kills)
      io.wp(i).iresp.valid := r_valids(numStages - 1) && !IsKilledByBranch(io.rp(i).brupdate, r_uops(numStages - 1))
      io.wp(i).iresp.bits.predicated := false.B
      io.wp(i).iresp.bits.uop := r_uops(numStages - 1)
      io.wp(i).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(i).brupdate, r_uops(numStages - 1))


    } else {
      require(numStages == 0)
      // pass req straight through to response

      // valid doesn't check kill signals, let consumer deal with it.
      // The LSU already handles it and this hurts critical path.
      io.wp(i).iresp.valid := io.rp(i).req.valid && !IsKilledByBranch(io.rp(i).brupdate, io.rp(i).req.bits.uop)
      io.wp(i).iresp.bits.predicated := false.B
      io.wp(i).iresp.bits.uop := io.rp(i).req.bits.uop
      io.wp(i).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(i).brupdate, io.rp(i).req.bits.uop)
    }

    val imul = Module(new PipelinedMultiplier(xLen, numStages))
    // request
    imul.io.req.valid := io.rp(i).req.valid
    imul.io.req.bits.fn := io.rp(i).req.bits.uop.ctrl.op_fcn
    imul.io.req.bits.dw := io.rp(i).req.bits.uop.ctrl.fcn_dw
    imul.io.req.bits.in1 := io.rp(i).req.bits.rs1_data
    imul.io.req.bits.in2 := io.rp(i).req.bits.rs2_data
    imul.io.req.bits.tag := DontCare
    // response
    io.wp(i).iresp.bits.data := imul.io.resp.bits.data
  }

}