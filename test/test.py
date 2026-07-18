# SPDX-FileCopyrightText: © 2026 Ruhan Gupta
# SPDX-License-Identifier: Apache-2.0
#
# Minimal bring-up tests for tt_um_fatigue_monitor.
# No MAX30102 model here -- these only check that reset behaves and that
# the sensor-fault path correctly detects "no sensor present" (SDA/SCL held
# low, i.e. not idle-high). Full functional testing needs a real or
# simulated MAX30102 and is done separately.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

CLOCK_PERIOD_NS = 100  # 10 MHz, matching tick_generator's assumed clock


async def start(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0  # SDA=0, SCL=0: no sensor / bus not idle-high
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)


def uo(dut):
    return dut.uo_out.value.to_unsigned()


@cocotb.test()
async def test_reset_state(dut):
    await start(dut)
    # bit4 = calibrating_init (1 on reset), everything else 0
    assert uo(dut) == 0x10, f"unexpected uo_out right after reset: {uo(dut):#x}"


@cocotb.test()
async def test_still_initializing_early(dut):
    await start(dut)
    await ClockCycles(dut.clk, 1000)
    val = uo(dut)
    assert (val >> 4) & 1 == 1, "should still be in sensor-init phase this early"
    assert (val >> 1) & 1 == 0, "sensor_fault shouldn't have tripped yet"


@cocotb.test()
async def test_sensor_fault_detected_with_no_sensor(dut):
    # Each failed transaction costs several I2C byte-level clock-stretch
    # timeouts (~1600 cycles each) before the controller gives up and
    # retries; FAULT_RETRY_LIMIT=8 attempts are needed before sensor_fault
    # asserts, so this needs a generous cycle budget, not just a few
    # thousand cycles.
    await start(dut)
    await ClockCycles(dut.clk, 150000)
    val = uo(dut)
    assert (val >> 1) & 1 == 1, "sensor_fault should assert with no sensor present"
