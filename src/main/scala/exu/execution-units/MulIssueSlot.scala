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

class MulIssueSlotIO()(implicit p: Parameters) extends BoomBundle
{
  val valid         = Output(Bool())
  val will_be_valid = Output(Bool()) // TODO code review, do we need this signal so explicitely?
  val request       = Output(Bool())
  val grant         = Input(Bool())

  val brupdate      = Input(new BrUpdateInfo())
  val kill          = Input(Bool()) // pipeline flush
  val clear         = Input(Bool()) // entry being moved elsewhere (not mutually exclusive with grant)

  val in_uop        = Flipped(Valid(new MicroOp())) // if valid, this WILL overwrite an entry!
  val out_uop       = Output(new MicroOp()) // the updated slot uop; will be shifted upwards in a collasping queue.
  val uop           = Output(new MicroOp()) // the current Slot's uop. Sent down the pipeline when issued.
}

class MulIssueSlot()(implicit p: Parameters) extends BoomModule {
  val io = IO(new MulIssueSlotIO)

  val s_invalid :: s_valid_1 :: s_valid_2 :: s_valid_3 :: s_valid_4 :: Nil = Enum(5)
  // slot invalid?
  // slot is valid, holding 1 uop
  def is_invalid = state === s_invalid
  def is_valid = state =/= s_invalid

  val next_state      = Wire(UInt()) // the next state of this slot (which might then get moved to a new slot)
  val next_uopc       = Wire(UInt()) // the next uopc of this slot (which might then get moved to a new slot)

  val state = RegInit(s_invalid)

  val tag = RegInit(0.U(2.W))
  val data = RegInit(VecInit(Seq.fill(4)(0.U(33.W))))
  val request = RegInit(0.U(4.W))
  val pattern = RegInit(0.U(2.W))

  val slot_uop = RegInit(NullMicroOp)
  val next_uop = Mux(io.in_uop.valid, io.in_uop.bits, slot_uop)

  //-----------------------------------------------------------------------------
  // next slot state computation
  // compute the next state for THIS entry slot (in a collasping queue, the
  // current uop may get moved elsewhere, and a new uop can enter

  when (io.kill) {
    state := s_invalid
  } .elsewhen (io.in_uop.valid) {
    state := io.in_uop.bits.iw_state  // need to initialized in issue unit
  } .elsewhen (io.clear) {
    state := s_invalid
  } .otherwise {
    state := next_state
  }

  //-----------------------------------------------------------------------------
  // "update" state
  // compute the next state for the micro-op in this slot. This micro-op may
  // be moved elsewhere, so the "next_state" travels with it.

  // defaults
  next_state := state
  next_uopc := slot_uop.uopc

  when (io.kill) {
    next_state := s_invalid
  } .elsewhen ((io.grant && (state === s_valid_1))) {
    // try to issue this uop.
    next_state := s_invalid
  }

  when (io.in_uop.valid) {
    slot_uop := io.in_uop.bits
    assert (is_invalid || io.clear || io.kill, "trying to overwrite a valid issue slot.")
  }

  // Handle branch misspeculations
  val next_br_mask = GetNewBrMask(io.brupdate, slot_uop)

  // was this micro-op killed by a branch? if yes, we can't let it be valid if
  // we compact it into an other entry
  when (IsKilledByBranch(io.brupdate, slot_uop)) {
    next_state := s_invalid
  }

  when (!io.in_uop.valid) {
    slot_uop.br_mask := next_br_mask
  }

  //-------------------------------------------------------------
  // Request Logic
  io.request := is_valid && !io.kill

  when (state === s_valid_1) {
    io.request := !io.kill
  }.otherwise {
    io.request := false.B
  }

  //assign outputs
  io.valid := is_valid
  io.uop := slot_uop


  // micro-op will vacate due to grant.
  val may_vacate = io.grant && ((state === s_valid_1))
  io.will_be_valid := is_valid && !may_vacate

  io.out_uop            := slot_uop
  io.out_uop.iw_state   := next_state
  io.out_uop.uopc       := next_uopc

  io.out_uop.br_mask    := next_br_mask

}
