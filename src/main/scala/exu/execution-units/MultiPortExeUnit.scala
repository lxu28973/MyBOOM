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
import boom.util.{BoomCoreStringPrefix, BranchKillableQueue, GetNewBrMask, ImmGen, IsKilledByBranch, spar2ExeSpar}
import freechips.rocketchip.util.UIntToAugmentedUInt
import freechips.rocketchip.rocket.ALU.{FN_MUL, FN_MULH, FN_MULHSU, FN_MULHU}
import freechips.rocketchip.rocket.{BP, DecodeLogic, N, PipelinedMultiplier, X, Y}


class MultiPortExeUnitIOr(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle {
  val fu_types = Output(Bits(FUC_SZ.W))
  val req = Flipped(new DecoupledIO(new FuncUnitReq(dataWidth)))
  val brupdate = Input(new BrUpdateInfo())
}

class MultiPortExeUnitIOw(val dataWidth: Int)(implicit p: Parameters) extends BoomBundle {
  val iresp = new DecoupledIO(new ExeUnitResp(dataWidth))
}

class MultiPortExeUnit(
                  val dataWidth: Int,
                  val numReadPort: Int,
                  val numWritePort: Int
                )(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle() {
    val rp = Vec(numReadPort, new MultiPortExeUnitIOr(dataWidth))
    val wp = Vec(numWritePort, new MultiPortExeUnitIOw(dataWidth))
  })

  for (i <- 0 until numWritePort){
    io.wp(i).iresp.bits.fflags.valid := false.B
    io.wp(i).iresp.bits.predicated := false.B
    io.wp(i).iresp.bits.uop.pdst_spar := VecInit((0 until 4).map(i => (io.wp(i).iresp.bits.data((i+1)*dataWidth/4 -1, i*dataWidth/4) === 0.U)))
    assert(io.wp(i).iresp.ready)
  }

  for (i <- 0 until numReadPort){
    io.rp(i).fu_types := Mux(io.rp(i).req.ready, FU_MUL, 0.U)
    io.rp(i).req.bits.uop.prs1_exe_spar :=  spar2ExeSpar(io.rp(i).req.bits.uop.prs1_spar)
    io.rp(i).req.bits.uop.prs2_exe_spar := spar2ExeSpar(io.rp(i).req.bits.uop.prs2_spar)
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

class RsPair(dataWidth: Int = 64) extends Bundle {
  val rs1_data = SInt((dataWidth/4 + 1).W)
  val rs2_data = SInt((dataWidth/4 + 1).W)
}

class MulArray(numMul: Int = 8) extends Module {
  val io = IO(new Bundle{
    val in = Vec(numMul, new RsPair)
    val out = Vec(numMul, SInt(34.W))
  })
  for (i <- 0 until numMul) {
    io.out(i) := io.in(i).rs1_data * io.in(i).rs2_data
  }
}

class AddArray extends Module {
  val io = IO(new Bundle{
    val in = Vec(8, SInt(128.W))
    val ctl = UInt(3.W)
    val out = Vec(2, SInt(128.W))
    val cache_out = SInt(128.W)
  })
  val sels = UIntToOH(io.ctl, 9)

  val internal = Vec(9, SInt(128.W))
  internal(0) := io.in(0)
  for (i <- 1 to 3) {
    internal(i) := Mux(sels(i), 0.S, internal(i - 1)) + io.in(i)
  }
  internal(4) := internal(3) + internal(5)
  for (i <- 5 to 7) {
    internal(i) := Mux(sels(i), 0.S, internal(i + 1)) + io.in(i - 1)
  }
  internal(8) := io.in(7)

  io.cache_out := internal(4)

  when(sels(0)){
    io.out(0) := 0.S
    io.out(1) := internal(4)
  }.elsewhen(sels(8)){
    io.out(0) := internal(4)
    io.out(1) := 0.S
  }.elsewhen(sels(4)){
    io.out(0) := internal(3)
    io.out(1) := internal(5)
  }.elsewhen(io.ctl < 4.U){
    io.out(0) := internal(io.ctl - 1.U)
    io.out(1) := internal(4)
  }.otherwise{
    io.out(0) := internal(4)
    io.out(1) := internal(io.ctl + 1.U)
  }
}

class Mulv2ExeUnit(
                  numReadPort: Int = 2,
                  numWritePort: Int = 2,
                  dataWidth: Int = 64
                  )(implicit p: Parameters)
  extends MultiPortExeUnit(
    dataWidth = dataWidth,
    numReadPort = numReadPort,
    numWritePort = numWritePort
  )
{

  val r_valids = RegNext(false.B)
  val r_uops   = RegNext(new MicroOp)

  val rs_pairs = Vec(2, Vec(4, Vec(4, new RsPair(dataWidth))))
  val p1_need_mul = Mux(io.rp(0).req.valid, io.rp(0).req.bits.uop.prs1_exe_spar * io.rp(0).req.bits.uop.prs2_exe_spar, 0.U)
  val p2_need_mul = Mux(io.rp(1).req.valid, io.rp(1).req.bits.uop.prs1_exe_spar * io.rp(1).req.bits.uop.prs2_exe_spar, 0.U)
  val p1_need_2cycles = p1_need_mul > 8.U
  val p2_need_2cycles = p2_need_mul > 8.U
  val need_2cycles = p1_need_2cycles || p2_need_2cycles
  val cache_rs1_data = RegNext(0.U(64.W))
  val cache_rs2_data = RegNext(0.U(64.W))
  val cache_rs1_eff = r_uops.prs1_exe_spar
  val cache_rs2_eff = r_uops.prs2_exe_spar
  val cache_op_fcn = r_uops.ctrl.op_fcn
  val cache_rst_data = RegNext(0.S(128.W))
  val use_cache = RegNext(false.B)
  when(need_2cycles){
    use_cache := true.B
  }
  assert(!(need_2cycles && use_cache), "crash, can't meet two continue need_2cycles")

  val p1_rs1_eff = io.rp(0).req.bits.uop.prs1_exe_spar
  val p1_rs2_eff = io.rp(0).req.bits.uop.prs2_exe_spar
  val p2_rs1_eff = io.rp(1).req.bits.uop.prs1_exe_spar
  val p2_rs2_eff = io.rp(1).req.bits.uop.prs2_exe_spar
  when(p1_need_2cycles){
    cache_rs1_data := io.rp(0).req.bits.rs1_data
    cache_rs2_data := io.rp(0).req.bits.rs2_data
    // handle incoming request
    r_valids := io.rp(0).req.valid && !IsKilledByBranch(io.rp(0).brupdate, io.rp(0).req.bits.uop) && !io.rp(0).req.bits.kill
    r_uops   := io.rp(0).req.bits.uop
    r_uops.br_mask := GetNewBrMask(io.rp(0).brupdate, io.rp(0).req.bits.uop)
  }.elsewhen(p2_need_2cycles){
    cache_rs1_data := io.rp(1).req.bits.rs1_data
    cache_rs2_data := io.rp(1).req.bits.rs2_data
    // handle incoming request
    r_valids := io.rp(1).req.valid && !IsKilledByBranch(io.rp(1).brupdate, io.rp(1).req.bits.uop) && !io.rp(1).req.bits.kill
    r_uops   := io.rp(1).req.bits.uop
    r_uops.br_mask := GetNewBrMask(io.rp(1).brupdate, io.rp(1).req.bits.uop)
  }

  val cmdHi = Vec(2, Bool())
  val lhsSigned = Vec(2, Bool())
  val rhsSigned = Vec(2, Bool())
  val cmdHalf = Vec(2, Bool())

  for (p <- 0 to 1) {
    val op_fcn = io.rp(p).req.bits.uop.ctrl.op_fcn
    val rs1_data = io.rp(p).req.bits.rs1_data
    val rs2_data = io.rp(p).req.bits.rs2_data
    if (p == 1) {
      when(use_cache){
        op_fcn := cache_op_fcn
        rs1_data := cache_rs1_data
        rs2_data := cache_rs2_data
      }
    }
    val decode = List(
      FN_MUL    -> List(N, X, X),
      FN_MULH   -> List(Y, Y, Y),
      FN_MULHU  -> List(Y, N, N),
      FN_MULHSU -> List(Y, Y, N))
    val cmdHi_ :: lhsSigned_ :: rhsSigned_ :: Nil =
      DecodeLogic(op_fcn, List(X, X, X), decode).map(_.asBool)
    val cmdHalf_ = (dataWidth > 32).B && io.rp(p).req.bits.uop.ctrl.fcn_dw === DW_32  // for MULW instruction

    cmdHi(p) := cmdHi_
    lhsSigned(p) := lhsSigned_
    rhsSigned(p) := rhsSigned_
    cmdHalf(p) := cmdHalf_

    for (i <- 0 until 4) {
      for (j <- 0 until 4) {
        if (i == 3){
          rs_pairs(p)(i)(j).rs1_data := Cat(lhsSigned_ && rs1_data(dataWidth-1), rs1_data(dataWidth/4 * (i+1) - 1, dataWidth/4 * i)).asSInt
        }
        else {
          rs_pairs(p)(i)(j).rs1_data := Cat(0.U(1.W), rs1_data(dataWidth/4 * (i+1) - 1, dataWidth/4 * i)).asSInt
        }
        if (j == 3){
          rs_pairs(p)(i)(j).rs2_data := Cat(rhsSigned_ && rs2_data(dataWidth-1), rs2_data(dataWidth/4 * (j+1) - 1, dataWidth/4 * j)).asSInt
        }
        else {
          rs_pairs(p)(i)(j).rs2_data := Cat(0.U(1.W), rs2_data(dataWidth/4 * (j+1) - 1, dataWidth/4 * j)).asSInt
        }
      }
    }

  }
  val non_spar_table = Vec(2, Vec(4, Vec(4, Bool())))
  for (p <- 0 to 1) {
    val prs1_exe_spar = io.rp(p).req.bits.uop.prs1_exe_spar
    val prs2_exe_spar = io.rp(p).req.bits.uop.prs2_exe_spar

    for (i <- 0 until 4) {
      for (j <- 0 until 4) {
        non_spar_table(p)(i)(j) := (prs1_exe_spar > i.U && prs2_exe_spar > j.U)
        if (p == 1) {
          if (i < 2) {
            when(use_cache){
              non_spar_table(p)(i)(j) := false.B
            }
          }
          else if (i == 3 & j < 2) {
            when(use_cache && cache_rs1_eff === 3.U){
              non_spar_table(p)(i)(j) := false.B
            }
          }
        }
      }
    }
  }

  val mul_array = new MulArray
  for (i <- 0 until 8) {
    mul_array.io.in(i).rs1_data := 0.S
    mul_array.io.in(i).rs2_data := 0.S
  }

  val in = mul_array.io.in
  val out = 0.U asTypeOf Vec(8, SInt(128.W))
  var mul_ind = 0.U(4.W)
  for (p <- 0 to 1) {
    for (i <- 0 until 4) {
      for (j <- 0 until 4) {
        when(non_spar_table(p)(i)(j) && mul_ind < 8.U) {
          in(mul_ind) := rs_pairs(p)(i)(j)
          out(mul_ind) := mul_array.io.out(mul_ind) << ((i + j) * 16).U
        }
        mul_ind = Mux(non_spar_table(p)(i)(j) && mul_ind < 8.U, mul_ind + 1.U, mul_ind)
      }
    }
  }

  val add_array = new AddArray

  add_array.io.in := out
  add_array.io.ctl := p1_need_mul
  cache_rst_data := add_array.io.cache_out

  val add_out = add_array.io.out
  when(use_cache){
    add_out(1) := add_array.io.out(1) + cache_rst_data
  }
  val muxed = Wire(Vec(2, SInt(64.W)))
  for (p <- 0 to 1) {
    muxed(p) := Mux(cmdHi(p), add_out(p)(2*64-1, 64), Mux(cmdHalf(p), add_out(p)(32-1, 0).sextTo(64), add_out(p)(64-1, 0)))
  }

  // handle outgoing (branch could still kill it)
  // consumer must also check for pipeline flushes (kills)
  io.wp(0).iresp.valid    := Mux(p1_need_2cycles, false.B, io.rp(0).req.valid && !IsKilledByBranch(io.rp(0).brupdate, io.rp(0).req.bits.uop) && !io.rp(0).req.bits.kill)
  io.wp(0).iresp.bits.uop := io.rp(0).req.bits.uop
  io.wp(0).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(0).brupdate, io.rp(1).req.bits.uop)
  io.wp(0).iresp.bits.data := muxed(0)
  io.wp(1).iresp.valid    := Mux(p2_need_2cycles, false.B, Mux(use_cache, r_valids && !IsKilledByBranch(io.rp(1).brupdate, r_uops), io.rp(1).req.valid && !IsKilledByBranch(io.rp(1).brupdate, io.rp(1).req.bits.uop) && !io.rp(1).req.bits.kill))
  io.wp(1).iresp.bits.uop := Mux(use_cache, r_uops, io.rp(1).req.bits.uop)
  io.wp(1).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(1).brupdate, Mux(use_cache, r_uops, io.rp(1).req.bits.uop))
  io.wp(1).iresp.bits.data := muxed(1)
}


class MulExeUnit(
                numReadPort: Int = 4,
                numWritePort: Int = 10,
                dataWidth: Int = 64
                )(implicit p: Parameters)
  extends MultiPortExeUnit(
    dataWidth,
    numReadPort,
    numWritePort
  )
{

  val delogic = Module(new MulDeLogic(numReadPort, dataWidth))

  for (i <- 0 until numReadPort) {
    delogic.io.rp(i).brupdate := io.rp(i).brupdate
    delogic.io.rp(i).req.bits := io.rp(i).req.bits
    delogic.io.rp(i).req.valid := io.rp(i).req.valid && !IsKilledByBranch(io.rp(i).brupdate, io.rp(i).req.bits.uop)
  }

  for (i <- 0 until numReadPort){
    when(delogic.io.zeroDetectOut(i).resZero){
      io.wp(i).iresp.bits.data := 0.U
      io.wp(i).iresp.valid := io.rp(i).req.valid && !IsKilledByBranch(io.rp(i).brupdate, io.rp(i).req.bits.uop)
      io.wp(i).iresp.bits.uop := io.rp(i).req.bits.uop
      io.wp(i).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(i).brupdate, io.rp(i).req.bits.uop)
    }.otherwise{
      io.wp(i).iresp.valid := false.B
    }
  }

  val issuePart = Module(new MulIssuePart(numReadPort, dataWidth, 16))

  for (i <- 0 until numReadPort) {
    issuePart.io.in(i).valid := io.rp(i).req.valid && !delogic.io.zeroDetectOut(i).resZero && !IsKilledByBranch(io.rp(i).brupdate, io.rp(i).req.bits.uop)
    issuePart.io.in(i).dec := delogic.io.decOut(i)
    issuePart.io.in(i).zeroDetect := delogic.io.zeroDetectOut(i)
    issuePart.io.in(i).rs1_data := io.rp(i).req.bits.rs1_data
    issuePart.io.in(i).rs2_data := io.rp(i).req.bits.rs2_data
    issuePart.io.in(i).uop := io.rp(i).req.bits.uop
    io.rp(i).req.ready := issuePart.io.in(i).ready
  }

  issuePart.io.brupdate := io.rp(0).brupdate
  issuePart.io.kill := io.rp(0).req.bits.kill

  val packet = Wire(Vec(numReadPort, Valid(new Packet)))
  packet := issuePart.io.packet

  val prodArray = RegInit(0.U asTypeOf Vec(3, Vec(4, SInt(65.W))))    // also can opt to 64 65 65 64
  val validArray = RegInit(0.U asTypeOf Vec(3, Vec(4, UInt(1.W))))
  val patternArray = RegInit(0.U asTypeOf Vec(3, UInt(2.W)))
  val uopArray = RegInit(VecInit(Seq.fill(3)(NullMicroOp)))
  val cmdhiArray = RegInit(0.U asTypeOf Vec(3, Bool()))


  // Add
  val add1 = Module(new ADD1)
  val add2 = Module(new ADD2)
  // default
  add1.io.in1 := DontCare
  add1.io.in2 := DontCare
  add2.io.in1 := DontCare
  add2.io.in2 := DontCare
  add2.io.in3 := DontCare

  for (i <- 8 to 9) {
    io.wp(i).iresp.bits.data := DontCare
    io.wp(i).iresp.bits.uop := DontCare
    io.wp(i).iresp.valid := false.B
  }

  var add1_bsy = WireDefault(false.B)
  val add2_bsy = WireDefault(false.B)
  for (i <- 2 to 0 by -1) {   // this is unnecessary, because at most only 2 entry will be issue
    if (i == 2) add1_bsy := false.B
    when(patternArray(i) === 1.U && PopCount(validArray(i).asUInt) === 2.U){
      when(!add1_bsy){
        when(validArray(i).asUInt === "b0011".U){
          add1.io.in1 := prodArray(i)(0)
          add1.io.in2 := (prodArray(i)(1) << 32.U)
        }.elsewhen(validArray(i).asUInt === "b1100".U){
          add1.io.in1 := prodArray(i)(2) << 32.U
          add1.io.in2 := prodArray(i)(3) << 64.U
        }.elsewhen(validArray(i).asUInt === "b0101".U){
          add1.io.in1 := prodArray(i)(0)
          add1.io.in2 := (prodArray(i)(2) << 32.U)
        }.elsewhen(validArray(i).asUInt === "b1010".U){
          add1.io.in1 := prodArray(i)(1) << 32.U
          add1.io.in2 := prodArray(i)(3) << 64.U
        }
        io.wp(8).iresp.bits.data := Mux(cmdhiArray(i), add1.io.out(2*dataWidth-1, dataWidth), add1.io.out(dataWidth-1, 0))
        io.wp(8).iresp.bits.uop := uopArray(i)
        io.wp(8).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(0).brupdate, uopArray(i))
        io.wp(8).iresp.valid := true.B && !IsKilledByBranch(io.rp(0).brupdate, uopArray(i))
      }.otherwise{
        when(validArray(i).asUInt === "b0011".U){
          add2.io.in1 := prodArray(i)(0)
          add2.io.in2 := (prodArray(i)(1) << 32.U)
          add2.io.in3 := 0.S
        }.elsewhen(validArray(i).asUInt === "b1100".U){
          add2.io.in1 := prodArray(i)(2) << 32.U
          add2.io.in2 := prodArray(i)(3) << 64.U
          add2.io.in3 := 0.S
        }.elsewhen(validArray(i).asUInt === "b0101".U){
          add2.io.in1 := prodArray(i)(0)
          add2.io.in2 := (prodArray(i)(2) << 32.U)
          add2.io.in3 := 0.S
        }.elsewhen(validArray(i).asUInt === "b1010".U){
          add2.io.in1 := prodArray(i)(1) << 32.U
          add2.io.in2 := prodArray(i)(3) << 64.U
          add2.io.in3 := 0.S
        }
        io.wp(9).iresp.bits.data := Mux(cmdhiArray(i), add2.io.out(2*dataWidth-1, dataWidth), add2.io.out(dataWidth-1, 0))
        io.wp(9).iresp.bits.uop := uopArray(i)
        io.wp(9).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(0).brupdate, uopArray(i))
        io.wp(9).iresp.valid := true.B && !IsKilledByBranch(io.rp(0).brupdate, uopArray(i))
        add2_bsy := true.B
      }
      validArray(i) := 0.U(4.W).asBools
    }.elsewhen(patternArray(i) === 2.U && PopCount(validArray(i).asUInt) === 4.U){
      assert(!add2_bsy)
      add2.io.in1 := Cat(prodArray(i)(3)(63,0), prodArray(i)(0)(63,0)).asSInt
      add2.io.in2 := prodArray(i)(1) << 32.U
      add2.io.in3 := prodArray(i)(2) << 32.U
      io.wp(9).iresp.bits.data := Mux(cmdhiArray(i), add2.io.out(2*dataWidth-1, dataWidth), add2.io.out(dataWidth-1, 0))
      io.wp(9).iresp.bits.uop := uopArray(i)
      io.wp(9).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(0).brupdate, uopArray(i))
      io.wp(9).iresp.valid := true.B && !IsKilledByBranch(io.rp(0).brupdate, uopArray(i))
      validArray(i) := 0.U(4.W).asBools
    }
    add1_bsy = (patternArray(i) === 1.U && PopCount(validArray(i).asUInt) === 2.U) || add1_bsy
  }

  // Multiple
  val mul_res = Wire(Vec(numReadPort, SInt(65.W)))
  for (i <- 0 until numReadPort) {
    mul_res(i) := packet(i).bits.rs1_x * packet(i).bits.rs2_x
    when(packet(i).bits.pattern === 0.U && packet(i).valid){
      val data = Wire(SInt((2*dataWidth).W))
      when(packet(i).bits.weight === 0.U) {
        data := mul_res(i)
      }.elsewhen(packet(i).bits.weight === 3.U){
        data := mul_res(i) << dataWidth.U
      }.otherwise{
        data := mul_res(i) << (dataWidth/2).U
      }
      io.wp(i + numReadPort).iresp.valid := packet(i).valid && !IsKilledByBranch(io.rp(0).brupdate, packet(i).bits.uop)
      io.wp(i + numReadPort).iresp.bits.uop := packet(i).bits.uop
      io.wp(i + numReadPort).iresp.bits.uop.br_mask := GetNewBrMask(io.rp(i).brupdate, packet(i).bits.uop)
      io.wp(i + numReadPort).iresp.bits.data := Mux(packet(i).bits.cmd_hi, data(2*dataWidth-1, dataWidth), Mux(packet(i).bits.cmd_half, data(dataWidth/2-1, 0).sextTo(dataWidth), data(dataWidth-1, 0)))
    }.otherwise{
      when(packet(i).valid){
        prodArray(packet(i).bits.tag)(packet(i).bits.weight) := mul_res(i)
        validArray(packet(i).bits.tag)(packet(i).bits.weight) := packet(i).valid && !IsKilledByBranch(io.rp(i).brupdate, packet(i).bits.uop) && !io.rp(i).req.bits.kill
        patternArray(packet(i).bits.tag) := packet(i).bits.pattern
        uopArray(packet(i).bits.tag) := packet(i).bits.uop
        uopArray(packet(i).bits.tag).br_mask := GetNewBrMask(io.rp(i).brupdate, packet(i).bits.uop)
        cmdhiArray(packet(i).bits.tag) := packet(i).bits.cmd_hi
      }
      io.wp(i + numReadPort).iresp.valid := false.B
    }
  }

}

class ADD1 extends Module {
  val io = IO(new Bundle() {
    val in1 = Input(SInt(128.W))
    val in2 = Input(SInt(128.W))
    val out = Output(SInt(128.W))
  })
  io.out := io.in1 + io.in2
}

class ADD2 extends Module {
  val io = IO(new Bundle() {
    val in1 = Input(SInt(128.W))
    val in2 = Input(SInt(128.W))
    val in3 = Input(SInt(128.W))
    val out = Output(SInt(128.W))
  })
  io.out := io.in1 + io.in2 + io.in3
}


// First use pipelined iMul to verify can add to BOOM
class MulExeUnitForTest(
                  numReadPort: Int = 4,
                  numWritePort: Int = 4
                )(implicit p: Parameters)
  extends MultiPortExeUnit(
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