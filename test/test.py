import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


@cocotb.test()
async def test_counter(dut):
    dut._log.info("Starting 8-bit counter test")

    # 1. Start 100 kHz clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # 2. Reset design
    dut.ena.value = 1
    dut.ui_in.value = 0b100  # oe=1, en=0, load=0
    dut.uio_in.value = 0
    dut.rst_n.value = 0      # Assert active-low reset

    await ClockCycles(dut.clk, 5)

    dut.rst_n.value = 1      # Release reset
    await RisingEdge(dut.clk)

    assert dut.uo_out.value == 0, \
        f"Expected 0 after reset, got {dut.uo_out.value}"

    # 3. Test Counting
    # oe=1, en=1, load=0 -> ui_in = 0b110
    dut.ui_in.value = 0b110

    await RisingEdge(dut.clk)  # Let enable take effect

    for expected in range(1, 6):
        await RisingEdge(dut.clk)

        assert dut.uo_out.value == expected, \
            f"Expected {expected}, got {dut.uo_out.value}"

    # 4. Test Parallel Load
    # oe=1, en=1, load=1 -> ui_in = 0b111
    dut.uio_in.value = 0xA5
    dut.ui_in.value = 0b111

    await RisingEdge(dut.clk)  # Single edge to latch 0xA5

    assert dut.uo_out.value == 0xA5, \
        f"Expected 0xA5 after load, got {hex(dut.uo_out.value)}"

    # Resume counting from 0xA5
    # oe=1, en=1, load=0 -> ui_in = 0b110
    dut.ui_in.value = 0b110

    await RisingEdge(dut.clk)

    assert dut.uo_out.value == 0xA6, \
        f"Expected 0xA6 on next count, got {hex(dut.uo_out.value)}"

    # 5. Test Output Enable / Tri-State
    # oe=0 -> output should be high impedance (Z)
    # ui_in = 0b010
    dut.ui_in.value = 0b010

    # Give combinational output assignment time to update
    await Timer(1, units="us")

    assert str(dut.uo_out.value) == "zzzzzzzz", \
        f"Expected high impedance (Z) when oe=0, got {dut.uo_out.value}"

    # Re-enable output
    # oe=1, en=1, load=0 -> ui_in = 0b110
    dut.ui_in.value = 0b110

    await Timer(1, units="us")

    # Counter should still contain 0xA6 because it was not clocked
    assert dut.uo_out.value == 0xA6, \
        f"Expected 0xA6 when output re-enabled, got {hex(dut.uo_out.value)}"

    # 6. Test Asynchronous Reset
    dut.rst_n.value = 0

    # Wait without a clock edge to verify asynchronous behavior
    await Timer(1, units="us")

    assert dut.uo_out.value == 0, \
        f"Expected 0 on async reset, got {dut.uo_out.value}"

    dut._log.info("All counter tests passed successfully!")