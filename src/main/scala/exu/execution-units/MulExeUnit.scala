package boom.exu


import scala.collection.mutable.{ArrayBuffer}

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{BP}
import freechips.rocketchip.tile.{XLen, RoCCCoreIO}
import freechips.rocketchip.tile

import FUConstants._
import boom.common._
import boom.ifu.{GetPCFromFtqIO}
import boom.util.{ImmGen, IsKilledByBranch, BranchKillableQueue, BoomCoreStringPrefix}

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



}