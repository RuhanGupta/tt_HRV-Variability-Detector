import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.accepted_interval.value = 0
    dut.accepted_interval_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def feed(dut, value, valid=True):
    """Drive one accepted_interval pulse for exactly one clock edge, then
    drop it low, with a settle edge afterward so the caller reads
    fully-resolved register values."""
    dut.accepted_interval.value = value
    dut.accepted_interval_valid.value = 1 if valid else 0
    await RisingEdge(dut.clk)
    dut.accepted_interval_valid.value = 0
    await RisingEdge(dut.clk)


async def feed_sequence(dut, values):
    """Feed a list of intervals one at a time, returning the
    variability_valid/current_variability observed after the LAST one."""
    for v in values:
        await feed(dut, v, valid=True)
    return int(dut.variability_valid.value), int(dut.current_variability.value)


@cocotb.test()
async def test_reset_clears_everything(dut):
    await start_clock(dut)
    await reset_dut(dut)
    assert dut.variability_valid.value == 0
    await RisingEdge(dut.clk)
    assert dut.variability_valid.value == 0


@cocotb.test()
async def test_first_interval_produces_no_output(dut):
    await start_clock(dut)
    await reset_dut(dut)

    await feed(dut, 90, valid=True)
    assert dut.variability_valid.value == 0, \
        "the very first accepted interval only establishes the reference"


@cocotb.test()
async def test_worked_example_from_spec(dut):
    """intervals [100,104,98,102,100,105,99,103,100] -> variability == 4"""
    await start_clock(dut)
    await reset_dut(dut)

    intervals = [100, 104, 98, 102, 100, 105, 99, 103, 100]
    valid, variability = await feed_sequence(dut, intervals)

    assert valid == 1, "variability_valid must pulse after the 9th interval"
    assert variability == 4, f"expected variability=4, got {variability}"


@cocotb.test()
async def test_constant_intervals_give_zero(dut):
    await start_clock(dut)
    await reset_dut(dut)

    valid, variability = await feed_sequence(dut, [90] * 9)
    assert valid == 1
    assert variability == 0, f"expected variability=0 for constant intervals, got {variability}"


@cocotb.test()
async def test_max_swing_sequence(dut):
    """Alternate between the validator's own accepted extremes (30 and 200).
    Reference=30, then 200,30,200,30,200,30,200,30 -> 8 diffs of 170 each.
    sum = 8*170 = 1360, variability = 1360 // 8 = 170."""
    await start_clock(dut)
    await reset_dut(dut)

    intervals = [30, 200, 30, 200, 30, 200, 30, 200, 30]
    valid, variability = await feed_sequence(dut, intervals)
    assert valid == 1
    assert variability == 170, f"expected variability=170, got {variability}"


@cocotb.test()
async def test_two_blocks_back_to_back(dut):
    """previous_interval is NOT reset between variability blocks, only
    difference_sum/difference_count are. Because of that continuous
    chaining, only the very FIRST block ever needs a "spare" interval to
    establish a reference; every later block completes after exactly 8
    new accepted intervals, since the prior block's last interval already
    serves as the reference for the new block's first difference."""
    await start_clock(dut)
    await reset_dut(dut)

    block1 = [100, 104, 98, 102, 100, 105, 99, 103, 100]  # 9 values -> 4
    block2 = [90, 90, 90, 90, 90, 90, 90, 90]             # 8 values -> 1 (carries prev=100 in)

    valid1, var1 = await feed_sequence(dut, block1)
    assert valid1 == 1
    assert var1 == 4, f"first block: expected 4, got {var1}"

    valid2, var2 = await feed_sequence(dut, block2)
    assert valid2 == 1
    assert var2 == 1, f"second block: expected 1 (one diff of 10 from carryover, then seven 0s), got {var2}"


@cocotb.test()
async def test_reset_mid_block(dut):
    await start_clock(dut)
    await reset_dut(dut)

    # reference + 4 differences, partway into a block
    await feed_sequence(dut, [100, 104, 98, 102, 100])

    await reset_dut(dut)

    # next interval after reset must be treated as a fresh reference again
    await feed(dut, 90, valid=True)
    assert dut.variability_valid.value == 0, \
        "after reset mid-block, the next interval must establish a new reference, not continue the old block"


@cocotb.test()
async def test_ignored_when_valid_low(dut):
    """Garbage on accepted_interval while accepted_interval_valid is low
    must not be latched as the reference interval. This is a direct
    regression test for a bug where the reference-establishing branch
    fired on the very first post-reset cycle regardless of
    accepted_interval_valid, silently latching whatever default/garbage
    value was on accepted_interval as the first reference."""
    await start_clock(dut)
    await reset_dut(dut)

    # hold garbage on the data lines with valid low for several cycles
    dut.accepted_interval.value = 777
    dut.accepted_interval_valid.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    assert dut.dut.have_previous_interval.value == 0, \
        "have_previous_interval must not be set while accepted_interval_valid has never been high"

    # now run the real worked example through and check the EXACT value.
    # if 777 had been wrongly latched as the reference earlier, this
    # would NOT come out to exactly 4.
    intervals = [100, 104, 98, 102, 100, 105, 99, 103, 100]
    valid, variability = await feed_sequence(dut, intervals)
    assert valid == 1
    assert variability == 4, \
        f"expected the standard worked-example result of 4, got {variability}, " \
        f"suggesting garbage may have been latched as an earlier reference"


@cocotb.test()
async def test_variability_valid_clears_during_idle_cycles(dut):
    """After variability_valid pulses, it must drop back to 0 and STAY 0
    through subsequent idle cycles (accepted_interval_valid low), not
    just whenever the next accepted_interval_valid pulse happens to
    arrive. Real heartbeats are tens to hundreds of cycles apart, so if
    the default-to-0 only happens on cycles where accepted_interval_valid
    is high, this pulse would incorrectly stay high for that entire gap."""
    await start_clock(dut)
    await reset_dut(dut)

    intervals = [100, 104, 98, 102, 100, 105, 99, 103, 100]
    valid, variability = await feed_sequence(dut, intervals)
    assert valid == 1
    assert variability == 4

    # accepted_interval_valid is already low after feed_sequence's last
    # feed() call. Hold it low for several MORE idle cycles and confirm
    # variability_valid actually clears and stays cleared.
    dut.accepted_interval_valid.value = 0
    for i in range(10):
        await RisingEdge(dut.clk)
        assert dut.variability_valid.value == 0, \
            f"variability_valid stuck high {i+1} idle cycle(s) after the pulse"
