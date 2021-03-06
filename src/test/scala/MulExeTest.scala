
import scala.collection.mutable.ArrayBuffer
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.{BP, PipelinedMultiplier}
import freechips.rocketchip.tile.{RoCCCoreIO, XLen}
import freechips.rocketchip.tile
import boom.exu.FUConstants._
import boom.common._
import boom.exu._
import boom.ifu.GetPCFromFtqIO
import boom.util.{BoomCoreStringPrefix, BranchKillableQueue, GetNewBrMask, ImmGen, IsKilledByBranch}
import chisel3.stage.ChiselGeneratorAnnotation
import freechips.rocketchip.util.UIntToAugmentedUInt
import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import chipyard.My6WideBoomConfig

//(new chisel3.stage.ChiselStage).execute(
//  Array("-X", "verilog"),
//  Seq(ChiselGeneratorAnnotation(() => new )


class MulExeTest(c: MulExeUnit()(c: My6WideBoomConfig)) extends PeekPokeTester(c) {
  def f_mul(a: Long, b: Long): Long = {
    a * b
  }
  def f_mulh(a: Long, b: Long): Long = {
    (BigInt(a) * BigInt(b) >> 64).toLong
  }
  def f_mulhu(a: BigInt, b: BigInt): Long = {
    (a * b >> 64).toLong
  }
  def f_mulhsu(a: Long, b: BigInt): Long = {
    (BigInt(a) * b >> 64).toLong
  }
  def mulModel(numReadPort: Int = 4, numWritePort: Int = 10)(){}

}
