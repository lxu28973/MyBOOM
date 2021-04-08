//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util.{PopCount, log2Ceil}
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str
import FUConstants._
import boom.common._
import boom.util.spar2ExeSpar

/**
 * Specific type of issue unit
 *
 * @param params issue queue params
 * @param numWakeupPorts number of wakeup ports for the issue queue
 */
class IssueUnitCollapsing(
  params: IssueParams,
  numWakeupPorts: Int)
  (implicit p: Parameters)
  extends IssueUnit(params.numEntries, params.issueWidth, numWakeupPorts, params.iqType, params.dispatchWidth)
{
  //-------------------------------------------------------------
  // Figure out how much to shift entries by

  val maxShift = dispatchWidth
  val vacants = issue_slots.map(s => !(s.valid)) ++ io.dis_uops.map(_.valid).map(!_.asBool)
  val shamts_oh = Array.fill(numIssueSlots+dispatchWidth) {Wire(UInt(width=maxShift.W))}
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
  for (i <- 1 until numIssueSlots + dispatchWidth) {
    shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), maxShift)
  }

  //-------------------------------------------------------------

  // which entries' uops will still be next cycle? (not being issued and vacated)
  val will_be_valid = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid) ++
                      (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                        !dis_uops(i).exception &&
                                                        !dis_uops(i).is_fence &&
                                                        !dis_uops(i).is_fencei)

  val uops = issue_slots.map(s=>s.out_uop) ++ dis_uops.map(s=>s)
  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in_uop.valid := false.B    // default is not move to here
    issue_slots(i).in_uop.bits  := uops(i+1)
    for (j <- 1 to maxShift by 1) {
      when (shamts_oh(i+j) === (1 << (j-1)).U) {    // is this slot data move to here
        issue_slots(i).in_uop.valid := will_be_valid(i+j)
        issue_slots(i).in_uop.bits  := uops(i+j)
      }
    }
    issue_slots(i).clear        := shamts_oh(i) =/= 0.U   // has slot data move to here
  }

  //-------------------------------------------------------------
  // Dispatch/Entry Logic
  // did we find a spot to slide the new dispatched uops into?

  val will_be_available = (0 until numIssueSlots).map(i =>
                            (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid))
  val num_available = PopCount(will_be_available)
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := RegNext(num_available > w.U)
  }

  //-------------------------------------------------------------
  // Issue Select Logic

  // set default
  for (w <- 0 until issueWidth) {
    io.iss_valids(w) := false.B
    io.iss_uops(w)   := NullMicroOp
    // unsure if this is overkill
    io.iss_uops(w).prs1 := 0.U
    io.iss_uops(w).prs2 := 0.U
    io.iss_uops(w).prs3 := 0.U
    io.iss_uops(w).lrs1_rtype := RT_X
    io.iss_uops(w).lrs2_rtype := RT_X
  }

  val requests = issue_slots.map(s => s.request)
  val port_issued = Array.fill(issueWidth){Bool()}    // CAUTION: Arrays are mutable
  for (w <- 0 until issueWidth) {
    port_issued(w) = false.B
  }

  val need_wait = RegNext(false.B)
  val next_free_muls = RegNext(8.U)
  val free_muls = Wire(Vec(issueWidth, UInt(4.W)))
  free_muls(0) := next_free_muls

  port_issued(issueWidth - 1) = need_wait // FIXME: when need_wait, disable the last issue port. need to be mul port

  val iss_is_fu_mul = Wire(Vec(issueWidth, Bool()))
  val iss_mul_need_2_cycle = Wire(Vec(issueWidth, Bool()))
  val iss_need_muls = Wire(Vec(issueWidth, UInt(5.W)))
  for (w <- 0 until issueWidth) {
    iss_is_fu_mul(w) := false.B
    iss_need_muls(w) := 0.U
    iss_mul_need_2_cycle(w) := false.B
  }
  for (w <- 0 until(issueWidth - 1)) {
    free_muls(w+1) := Mux(io.iss_valids(w) && iss_is_fu_mul(w), Mux(iss_mul_need_2_cycle(w), 0.U, free_muls(w) - iss_need_muls(w)), free_muls(w))
    when(iss_is_fu_mul(w) && iss_mul_need_2_cycle(w)){
      need_wait := true.B
      next_free_muls := free_muls(w) * 2.U - iss_need_muls(w)
    }
  }

  // TODO: Adding the lower priority of no sparsity
  for (i <- 0 until numIssueSlots) {    // allow issue queue have internal vacant
    issue_slots(i).grant := false.B
    var uop_issued = false.B    // CAUTION: var, uop_issued = a | uop_issued, first uop_issued and second uop_issued are not the same
    val need_muls = PopCount(issue_slots(i).uop.prs1_spar.map(!_)) * PopCount(issue_slots(i).uop.prs2_spar.map(!_))   // FIXME: this is only for mul, not support packed mul
    val mul_need_2_cycle = need_muls > 8.U
//    val no_spar = issue_slots(i).uop.prs1_spar.asUInt === 0.U && issue_slots(i).uop.prs2_spar.asUInt === 0.U
    for (w <- 0 until issueWidth) {
      val is_fu_mul = issue_slots(i).uop.fu_code === FU_MUL
      val fu_match = (issue_slots(i).uop.fu_code & io.fu_types(w)) =/= 0.U
      val can_allocate = Mux(is_fu_mul,  (free_muls(w) >= need_muls || free_muls(w)(3)) && fu_match, fu_match)

      when (requests(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(i).grant := true.B
        io.iss_valids(w) := true.B
        io.iss_uops(w) := issue_slots(i).uop
        iss_is_fu_mul(w) := is_fu_mul
        iss_mul_need_2_cycle(w) := mul_need_2_cycle
        iss_need_muls(w) := need_muls
      }
      val was_port_issued_yet = port_issued(w)
      port_issued(w) = (requests(i) && !uop_issued && can_allocate) | port_issued(w)
      uop_issued = (requests(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }

}
