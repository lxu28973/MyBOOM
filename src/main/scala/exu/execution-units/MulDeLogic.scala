package boom.exu

import scala.collection.mutable.ArrayBuffer
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.{BP, DecodeLogic, N, PipelinedMultiplier, X, Y}
import freechips.rocketchip.tile.{RoCCCoreIO, XLen}
import freechips.rocketchip.tile
import FUConstants._
import boom.common._
import boom.ifu.GetPCFromFtqIO
import boom.util.{BoomCoreStringPrefix, BranchKillableQueue, GetNewBrMask, ImmGen, IsKilledByBranch}
import freechips.rocketchip.rocket.ALU.{FN_MUL, FN_MULH, FN_MULHSU, FN_MULHU}

class MulDecodeIO extends Bundle {
  val cmdHi = Bool()
  val lhsSigned = Bool()
  val rhsSigned = Bool()
  val cmdHalf = Bool()
}

class MulZeroDetectIO extends Bundle {
  val request = UInt(4.W)
  val resZero = Bool()
  val pattern = UInt(2.W)
  val tag = UInt(2.W)
}

class MulDeLogic(numReadPort: Int, dataWidth: Int)(implicit p: Parameters) extends BoomModule{
  val io = IO(new Bundle{
    val rp = Vec(numReadPort, new MultiPortExeUnitIOr(dataWidth))
    val decOut = Vec(numReadPort, Output(new MulDecodeIO))
    val zeroDetectOut = Vec(numReadPort, Output(new MulZeroDetectIO))
  })

  for (j <- 0 until numReadPort) {
    // decode
    val in = io.rp(j).req.bits.uop

    val decode = List(
      FN_MUL -> List(N, X, X),
      FN_MULH -> List(Y, Y, Y),
      FN_MULHU -> List(Y, N, N),
      FN_MULHSU -> List(Y, Y, N))
    val cmdHi :: lhsSigned :: rhsSigned :: Nil =
      DecodeLogic(in.ctrl.op_fcn, List(X, X, X), decode).map(_.asBool)
    val cmdHalf = (dataWidth > 32).B && in.ctrl.fcn_dw === DW_32 // for MULW instruction
//    when(cmdHalf){cmdHi := false.B}
    assert(!(cmdHalf & cmdHi))

    io.decOut(j).cmdHalf := cmdHalf
    io.decOut(j).cmdHi := cmdHi
    io.decOut(j).lhsSigned := lhsSigned
    io.decOut(j).rhsSigned := rhsSigned

    // zero detect
    val in1 = io.rp(j).req.bits.rs1_data
    val in2 = io.rp(j).req.bits.rs2_data

    when(in1 === 0.U | in2 === 0.U){
      io.zeroDetectOut(j).resZero := true.B
    }.otherwise{
      io.zeroDetectOut(j).resZero := false.B
    }

    val nonZero = Wire(Vec(4, Bool()))
    nonZero(0) := Mux(in1(dataWidth/2-1, 0) === 0.U, 0.U, 1.U)
    nonZero(1) := Mux(in1(dataWidth-1, dataWidth/2) === 0.U, 0.U, 1.U)
    nonZero(2) := Mux(in2(dataWidth/2-1, 0) === 0.U, 0.U, 1.U)
    nonZero(3) := Mux(in2(dataWidth-1, dataWidth/2) === 0.U, 0.U, 1.U)

//    val genReq = List(
//      BitPat("b?1?1") -> BitPat("b???1"),
//      BitPat("b1??1") -> BitPat("b??1?"),
//      BitPat("b?11?") -> BitPat("b?1??"),
//      BitPat("b1?1?") -> BitPat("b1???")
//    )
//    io.zeroDetectOut(j).request := DecodeLogic(nonZero.asUInt, BitPat("b????"), genReq)
    io.zeroDetectOut(j).request := Cat(nonZero(3) & nonZero(1), nonZero(2) & nonZero(1), nonZero(3) & nonZero(0), nonZero(2) & nonZero(0))
    when(cmdHalf){
      io.zeroDetectOut(j).request := "b0001".U
    }

//    var numReq = WireInit(0.U(1.W))
//    for (i <- 0 until(4))
//      numReq = io.zeroDetectOut(j).request(i) +& numReq
    val numReq = PopCount(io.zeroDetectOut(j).request)

    when(numReq === 4.U){
      io.zeroDetectOut(j).pattern := 2.U
    }.elsewhen(numReq === 2.U){
      io.zeroDetectOut(j).pattern := 1.U
    }.otherwise(
      io.zeroDetectOut(j).pattern := 0.U
    )
  }

  // generate Tag
  val cnt = RegInit(0.U(3.W))

  val a = VecInit(Seq.fill(numReadPort+1)(0.U((log2Ceil(numReadPort)+1).W)))
  for (i <- 0 until numReadPort) {
    io.zeroDetectOut(i).tag := (cnt +& a(i)) % 3.U
    when(io.rp(i).req.valid & io.zeroDetectOut(i).pattern =/= 0.U) {
      a(i + 1) := a(i) + 1.U
    }.otherwise(
      a(i + 1) := a(i)
    )
  }
  cnt := (cnt +& a(numReadPort)) % 3.U
}

class TagGen(w: Int) extends Module {
  val io = IO(new Bundle {
    val valid = Input(Vec(w, Bool()))
    val pattern = Input(Vec(w, UInt(2.W)))
    val tag = Output(Vec(w, UInt(2.W)))
  })

  import io._

  val cnt = RegInit(0.U(2.W))

  val a = VecInit(Seq.fill(w+1)(0.U(log2Ceil(w).W)))
  for (i <- 0 until w) {
    tag(i) := (cnt +& a(i)) % 3.U
    when(valid(i) & pattern(i) =/= 0.U) {
      a(i + 1) := a(i) + 1.U
    }.otherwise(
      a(i + 1) := a(i)
    )
  }

  cnt := (cnt +& a(w)) % 3.U
}