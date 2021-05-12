//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename BusyTable
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class BusyResp extends Bundle
{
  val prs1_busy = Bool()
  val prs2_busy = Bool()
  val prs3_busy = Bool()
  val prs1_spar = Vec(4, Bool())
  val prs2_spar = Vec(4, Bool())
}

class RenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val busy_resps = Output(Vec(plWidth, new BusyResp))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))
    val wb_spar = Input(Vec(numWbPorts, Vec(4, Bool())))

    val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
  })

  val busy_table = RegInit(0.U(numPregs.W))
  // Unbusy written back registers.
  val busy_table_wb = busy_table & ~(io.wb_pdsts zip io.wb_valids)
    .map {case (pdst, valid) => UIntToOH(pdst) & Fill(numPregs, valid.asUInt)}.reduce(_|_)
  // Rebusy newly allocated registers.
  val busy_table_next = busy_table_wb | (io.ren_uops zip io.rebusy_reqs)
    .map {case (uop, req) => UIntToOH(uop.pdst) & Fill(numPregs, req.asUInt)}.reduce(_|_)

  busy_table := busy_table_next

  val spar_table = RegInit(0.U asTypeOf Vec(numPregs, Vec(4, Bool())))
  // update situation of spar === 1
  val update_spar_1 = Wire(Vec(numWbPorts, Vec(4, Bool())))
  for (i <- 0 until numWbPorts) {
    (0 to 3).map(j => update_spar_1(i)(j) := io.wb_valids(i) & io.wb_spar(i)(j))
  }

  val spar_table_wb = Wire(Vec(4, UInt(numPregs.W)))
  for (j <- 0 until 4) {
    var spar_ = Fill(numPregs, update_spar_1(0)(j).asUInt) & UIntToOH(io.wb_pdsts(0))
    for (i <- 1 until numWbPorts) {
      spar_ = spar_ | (Fill(numPregs, update_spar_1(i)(j).asUInt) & UIntToOH(io.wb_pdsts(i)))
    }
    spar_table_wb(j) := spar_
  }

  val spar_table_next = WireDefault(0.U asTypeOf Vec(numPregs, Vec(4, Bool())))
  // Update sparsity flag
  for (i <- 0 until numPregs) {
    for (j <- 0 until 4) {
      spar_table_next(i)(j) := spar_table(i)(j) | spar_table_wb(j)(i)
    }
  }

  // update situation of spar === 0
  val update_spar_0 = Wire(Vec(numWbPorts, Vec(4, Bool())))
  for (i <- 0 until numWbPorts) {
    (0 to 3).map(j => update_spar_0(i)(j) := (!io.wb_valids(i)) | io.wb_spar(i)(j))
  }

  val spar_table_wb_0 = Wire(Vec(4, UInt(numPregs.W)))
  for (j <- 0 until 4) {
    var spar_ = Fill(numPregs, update_spar_0(0)(j).asUInt) | (~(UIntToOH(io.wb_pdsts(0))))
    for (i <- 1 until numWbPorts) {
      spar_ = spar_ & (Fill(numPregs, update_spar_0(i)(j).asUInt) | (~(UIntToOH(io.wb_pdsts(i)))))
    }
    spar_table_wb_0(j) := spar_
  }

  // Update sparsity flag
  for (i <- 0 until numPregs) {
    for (j <- 0 until 4) {
      spar_table(i)(j) := spar_table_next(i)(j) & spar_table_wb_0(j)(i)
    }
  }

  spar_table(0) := VecInit(Seq.fill(4)(true.B))

  // Read the busy table.
  for (i <- 0 until plWidth) {
    val prs1_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs1 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs2_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs3_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)

    io.busy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) || prs1_was_bypassed && bypass.B
    io.busy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) || prs2_was_bypassed && bypass.B
    io.busy_resps(i).prs3_busy := busy_table(io.ren_uops(i).prs3) || prs3_was_bypassed && bypass.B
    if (!float) io.busy_resps(i).prs3_busy := false.B
    io.busy_resps(i).prs1_spar := spar_table(io.ren_uops(i).prs1)
    io.busy_resps(i).prs2_spar := spar_table(io.ren_uops(i).prs2)
  }

  io.debug.busytable := busy_table
}
