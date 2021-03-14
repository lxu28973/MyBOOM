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
import freechips.rocketchip.util.UIntToAugmentedUInt

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
    assert(io.wp(i).iresp.ready)
  }

  for (i <- 0 until numReadPort){
    io.rp(i).fu_types := Mux(io.rp(i).req.ready, FU_MUL, 0.U)
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