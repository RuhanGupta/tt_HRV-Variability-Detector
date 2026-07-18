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
    # sensor_fault pulses periodically (~5,000 cycles high out of a
    # ~65,600-cycle retry period: 2,000-cycle power-wait + 8 failed
    # init attempts, each costing ~7,300 cycles due to I2C clock-stretch
    # timeouts with no sensor pulling SCL high). Poll well past one full
    # period so the result doesn't depend on which phase we start in.
    await start(dut)
    seen_fault = False
    for _ in range(200000):
        await ClockCycles(dut.clk, 1)
        if (uo(dut) >> 1) & 1:
            seen_fault = True
            break
    assert seen_fault, "sensor_fault never asserted within the poll window"
