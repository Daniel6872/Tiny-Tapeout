import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_counter(dut):
    dut._log.info("Starting 8-bit counter test")

    # Start 100 kHz clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Initialize inputs
    dut.ena.value = 1
    dut.ui_in.value = 0   # load=0, en=0, oe=0
    dut.uio_in.value = 0  # data_in = 0x00
    dut.rst_n.value = 0   # Assert active-low reset
    
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1   # Release reset
    await ClockCycles(dut.clk, 2)

    # Test Counting (oe=1, en=1, load=0 -> ui_in = 0b110 = 6)
    dut.ui_in.value = 0b110
    await ClockCycles(dut.clk, 5)
    assert dut.uo_out.value == 5, f"Expected 5, got {dut.uo_out.value}"

    # Test Parallel Load (oe=1, en=1, load=1 -> ui_in = 0b111 = 7)
    dut.uio_in.value = 0xA5
    dut.ui_in.value = 0b111
    await ClockCycles(dut.clk, 1)
    
    # Resume counting from 0xA5
    dut.ui_in.value = 0b110
    await ClockCycles(dut.clk, 1)
    assert dut.uo_out.value == 0xA6, f"Expected 0xA6, got {hex(dut.uo_out.value)}"

    # Test Async Reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 1)
    assert dut.uo_out.value == 0, f"Expected 0 on reset, got {dut.uo_out.value}"

    dut._log.info("All counter tests passed successfully!")