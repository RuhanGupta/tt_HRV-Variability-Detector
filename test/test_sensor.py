# SPDX-FileCopyrightText: © 2026 Ruhan Gupta
# SPDX-License-Identifier: Apache-2.0
#
# Behavioural MAX30102 I2C slave model + end-to-end FIFO sample-read test.
#
# WHY THIS EXISTS
# ---------------
# The bring-up tests in test.py have no MAX30102 model: they only check reset
# and the no-sensor fault path. They never exercise the FIFO-pointer
# acquisition loop (S_RUN_READ_WRPTR -> S_RUN_READ_RDPTR -> S_RUN_CHECK_AVAIL
# -> S_RUN_READ_SAMPLE), which is exactly where the wr_ptr-capture bug lived.
#
# The slave below is CLOCK-SYNCHRONISED: it samples the two I2C lines on every
# rising edge of the DUT clock and reacts to SCL/SDA edges. That tracks the
# tick-based master exactly without fragile free-running timers.
#
# Open-drain convention (matches the RTL top level):
#   uio_oe[0] = sda_drive_low  -> master pulls SDA low when oe[0]=1
#   uio_oe[1] = scl_drive_low  -> master pulls SCL low when oe[1]=1
#   uio_out[1:0] are tied 0.  uio_in[0]=SDA level, uio_in[1]=SCL level (fed back).
# The slave only ever pulls a line low or releases it; never drives high.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLOCK_PERIOD_NS = 100  # 10 MHz

SLAVE_ADDR_W = 0xAE
SLAVE_ADDR_R = 0xAF
REG_FIFO_WR_PTR = 0x04
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA = 0x07
REG_MODE_CONFIG = 0x09
REG_PART_ID = 0xFF
PART_ID_EXPECT = 0x15


class MAX30102Model:
    """Clock-synchronised bit-level I2C slave emulating the MAX30102."""

    def __init__(self, dut):
        self.dut = dut
        self.sda_low = False

        self.samples = [0x0155AA, 0x02AA55, 0x013333, 0x02CCCC, 0x018001]
        self._sample_idx = 0
        self._bytes_of_sample = 0
        self.wr_ptr = 0
        self.rd_ptr = 0
        self._wrptr_reads = 0
        self.samples_served = 0

        self._prev_scl = 1
        self._prev_sda = 1
        self.bitcnt = 0
        self.shift = 0
        self.phase = "IDLE"
        self.pending_reg = REG_PART_ID
        self.tx_byte = 0
        self.tx_bits_left = 0
        self._ack_next_phase = "IDLE"
        self._ack_high_seen = False

    # ---- line sensing / driving -------------------------------------
    def _master_sda_low(self):
        return bool(int(self.dut.uio_oe.value) & 0x1)

    def _master_scl_low(self):
        return bool(int(self.dut.uio_oe.value) & 0x2)

    def sda(self):
        return 0 if (self._master_sda_low() or self.sda_low) else 1

    def scl(self):
        return 0 if self._master_scl_low() else 1

    def _drive(self):
        v = int(self.dut.uio_in.value) & ~0x3
        v |= (self.sda() & 1) | ((self.scl() & 1) << 1)
        self.dut.uio_in.value = v

    def _pull_sda(self):
        self.sda_low = True
        self._drive()

    def _release_sda(self):
        self.sda_low = False
        self._drive()

    # ---- FIFO / registers -------------------------------------------
    def _next_sample_byte(self):
        s = self.samples[min(self._sample_idx, len(self.samples) - 1)]
        b = [(s >> 16) & 0x03, (s >> 8) & 0xFF, s & 0xFF][self._bytes_of_sample]
        self._bytes_of_sample += 1
        if self._bytes_of_sample == 3:
            self._bytes_of_sample = 0
            self._sample_idx += 1
            self.samples_served += 1
        return b

    def _read_reg(self, addr):
        if addr == REG_FIFO_WR_PTR:
            self._wrptr_reads += 1
            if self._wrptr_reads <= len(self.samples):
                self.wr_ptr = (self.wr_ptr + 1) & 0x1F
            return self.wr_ptr
        if addr == REG_FIFO_RD_PTR:
            return self.rd_ptr
        if addr == REG_MODE_CONFIG:
            return 0x00
        if addr == REG_PART_ID:
            return PART_ID_EXPECT
        return 0x00

    # ---- main clocked decode loop -----------------------------------
    async def run(self):
        self._drive()
        while True:
            await RisingEdge(self.dut.clk)
            # Always refresh the fed-back bus state first: uio_in must mirror the
            # wired-AND of master + slave on BOTH lines every cycle. SCL is driven
            # only by the master, so if we refreshed only when touching SDA the
            # SCL feedback bit would go stale and the master's S_BIT_RISE would
            # hang waiting for a high it can't see.
            self._drive()
            scl = self.scl()
            sda = self.sda()

            scl_rise = (self._prev_scl == 0 and scl == 1)
            scl_fall = (self._prev_scl == 1 and scl == 0)
            scl_steady_high = (self._prev_scl == 1 and scl == 1)

            # START / STOP: an SDA transition while SCL is steady-high. We only
            # honour these when the slave is NOT actively driving the bus and is
            # at a byte boundary. Otherwise a legitimate data/ACK bit (SDA moving
            # while SCL happens to be high across a clock) would be misread as a
            # STOP/START -- e.g. the all-ones 0xFF register address.
            can_startstop = (not self.sda_low) and self.phase not in ("TX", "ACK_DRIVE")
            if scl_steady_high and can_startstop:
                if self._prev_sda == 1 and sda == 0:      # (repeated) START
                    self.phase = "ADDR"
                    self.bitcnt = 0
                    self.shift = 0
                    self.sda_low = False
                elif self._prev_sda == 0 and sda == 1:    # STOP
                    self.phase = "IDLE"
                    self._release_sda()

            if scl_rise:
                if self.phase in ("ADDR", "REG", "DATA_W"):
                    self.shift = ((self.shift << 1) | sda) & 0x1FF
                    self.bitcnt += 1
                    if self.bitcnt == 8:
                        # Byte complete. Decide ACK and pull SDA low NOW so it is
                        # already stable-low before the master opens its ACK
                        # sampling window (S_ACK_RISE -> S_ACK_HIGH).
                        self._on_byte_received(self.shift & 0xFF)
                        self.bitcnt = 0
                        self.shift = 0
                elif self.phase == "ACK_DRIVE":
                    # This rising edge IS the master's ACK bit sampling window
                    # (S_ACK_HIGH samples sda_in here). Mark it seen so we
                    # release only after this window closes.
                    self._ack_high_seen = True
                elif self.phase == "TX_ACK":
                    self._on_master_ack(sda == 0)

            if scl_fall:
                if self.phase == "ACK_DRIVE":
                    # Release only after the ACK bit's high pulse has been seen,
                    # i.e. after the master has actually sampled the low ACK.
                    if self._ack_high_seen:
                        self._ack_high_seen = False
                        self._release_sda()
                        self._after_ack()
                elif self.phase == "TX":
                    self._drive_next_tx_bit()

            self._prev_scl = scl
            self._prev_sda = sda

    # ---- handlers ----------------------------------------------------
    def _on_byte_received(self, byte):
        if self.phase == "ADDR":
            if byte == SLAVE_ADDR_W:
                self._begin_ack("REG")
            elif byte == SLAVE_ADDR_R:
                self._begin_ack("TX_START")
            else:
                self.phase = "IDLE"
        elif self.phase == "REG":
            self.pending_reg = byte
            self._begin_ack("DATA_W")
        elif self.phase == "DATA_W":
            self._begin_ack("REG_HOLD")

    def _begin_ack(self, next_phase):
        self._pull_sda()               # hold ACK low across the ACK high pulse
        self.phase = "ACK_DRIVE"
        self._ack_next_phase = next_phase

    def _after_ack(self):
        nxt = self._ack_next_phase
        if nxt == "TX_START":
            if self.pending_reg == REG_FIFO_DATA:
                self.tx_byte = self._next_sample_byte()
            else:
                self.tx_byte = self._read_reg(self.pending_reg)
            self.phase = "TX"
            self.tx_bits_left = 8
            self._drive_first_tx_bit()
        elif nxt == "REG_HOLD":
            self.phase = "REG_HOLD"
        else:
            self.phase = nxt

    def _drive_first_tx_bit(self):
        bit = (self.tx_byte >> 7) & 1
        self._release_sda() if bit else self._pull_sda()
        self.tx_bits_left = 7

    def _drive_next_tx_bit(self):
        if self.tx_bits_left > 0:
            bit = (self.tx_byte >> (self.tx_bits_left - 1)) & 1
            self._release_sda() if bit else self._pull_sda()
            self.tx_bits_left -= 1
        else:
            self._release_sda()        # let master drive ACK
            self.phase = "TX_ACK"

    def _on_master_ack(self, acked):
        if self.pending_reg == REG_FIFO_DATA and self._bytes_of_sample != 0:
            self.tx_byte = self._next_sample_byte()
            self.phase = "TX"
            self.tx_bits_left = 8
            self._drive_first_tx_bit()
        else:
            self.phase = "IDLE"
            self._release_sda()


async def _start(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0x03      # bus idle-high on SDA, SCL
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)


@cocotb.test()
async def test_fifo_sample_read_with_sensor_model(dut):
    """With a responding MAX30102 model the DUT must leave init, read the FIFO,
    and pulse sample_valid (uo_out[5]). This is the path the wr_ptr-capture fix
    repaired; before the fix fifo_wr_ptr stayed 0 and no sample was ever read."""
    await _start(dut)
    model = MAX30102Model(dut)
    cocotb.start_soon(model.run())

    saw_sample_valid = False
    for _ in range(600_000):
        await ClockCycles(dut.clk, 1)
        if (int(dut.uo_out.value) >> 5) & 1:
            saw_sample_valid = True
            break

    dut._log.info(
        f"samples served={model.samples_served}, wr_ptr reads={model._wrptr_reads}")
    assert model.samples_served > 0, (
        "sensor model never served a FIFO sample -- DUT did not reach "
        "S_RUN_READ_SAMPLE (wr_ptr likely stuck at 0)")
    assert saw_sample_valid, (
        "sample_valid (uo_out[5]) never pulsed -- FIFO read path did not "
        "complete a sample")


@cocotb.test()
async def test_wr_ptr_capture_advances_available(dut):
    """Tighter check on the specific bug: the DUT must poll WR_PTR repeatedly
    AND read multiple samples, proving fifo_available came from a freshly
    captured, non-zero fifo_wr_ptr. If the capture were still broken,
    fifo_available = (0 - rd_ptr) & 0x1F = 0 and the DUT would loop forever in
    S_RUN_POLL_WAIT, never requesting FIFO_DATA."""
    await _start(dut)
    model = MAX30102Model(dut)
    cocotb.start_soon(model.run())

    for _ in range(600_000):
        await ClockCycles(dut.clk, 1)
        if model.samples_served >= 2:
            break

    dut._log.info(
        f"samples served={model.samples_served}, wr_ptr reads={model._wrptr_reads}")
    assert model._wrptr_reads >= 2, (
        f"expected repeated WR_PTR polling, saw {model._wrptr_reads}")
    assert model.samples_served >= 2, (
        f"DUT read only {model.samples_served} sample(s); wr_ptr capture / "
        "fifo_available may not be advancing")
