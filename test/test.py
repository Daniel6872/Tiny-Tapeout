import os
import random

import cocotb
from cocotb.triggers import Timer

from test_butterfly import butterfly_reference


def fft_reference(samples):
    """Eight real samples, bit-reversed DIT loading, one-bit scaling per stage."""
    if len(samples) != 8 or any(not -64 <= value <= 63 for value in samples):
        raise ValueError("expected eight signed 7-bit samples")
    data = [(0, 0)] * 8
    for index, value in enumerate(samples):
        address = ((index & 1) << 2) | (index & 2) | ((index >> 2) & 1)
        data[address] = (value, 0)
    for size in (2, 4, 8):
        half = size // 2
        for base in range(0, 8, size):
            for offset in range(half):
                a, b = base + offset, base + offset + half
                data[a], data[b] = butterfly_reference(
                    data[a], data[b], offset * (8 // size), 1
                )
    return data


async def cycle(dut, sample=0, valid=0):
    dut.clk.value = 0
    dut.ui_in.value = sample & 255
    # Exercise all ignored input pins as well.
    dut.uio_in.value = 0xFE | valid
    await Timer(5, unit="us")
    dut.clk.value = 1
    await Timer(5, unit="us")
    assert int(dut.uio_oe.value) == 2
    assert int(dut.uio_out.value) & 0xFD == 0
    return int(dut.uo_out.value), (int(dut.uio_out.value) >> 1) & 1


async def reset(dut):
    dut.ena.value = 1
    dut.rst_n.value = 0
    assert await cycle(dut) == (0, 0)
    if os.getenv("GATES") != "yes":
        memory = dut.user_project.top_wrapper_inst.reg_file_inst.mem
        assert all(int(memory[index].value) == 0 for index in range(8))
    dut.rst_n.value = 1


async def frame(dut, samples, rng, gaps=True):
    for sample in samples:
        for _ in range(rng.randrange(4) if gaps else 0):
            assert await cycle(dut, rng.randrange(256), 0) == (0, 0)
        _, valid = await cycle(dut, (sample & 127) | rng.choice((0, 128)), 1)
        assert valid == 0

    # Inputs presented while busy must not load or extend computation.
    for _ in range(12):
        assert await cycle(dut, rng.randrange(256), 1) == (0, 0)

    expected = [component & 255 for bin_value in fft_reference(samples)
                for component in bin_value]
    for index, byte in enumerate(expected):
        actual, valid = await cycle(dut, rng.randrange(256), 1)
        assert valid == 1, f"missing valid byte {index} for {samples}"
        assert actual == byte, (
            f"samples={samples}, byte={index}: got {actual}, expected {byte}"
        )


@cocotb.test()
async def test_fft_frames(dut):
    await reset(dut)
    rng = random.Random(0xFF78)
    assert fft_reference([32] * 8) == [(32, 0)] + [(0, 0)] * 7
    assert fft_reference([32, 0, 0, 0, 0, 0, 0, 0]) == [(4, 0)] * 8
    assert fft_reference([32, -32] * 4) == [(0, 0)] * 4 + [(32, 0)] + [(0, 0)] * 3
    vectors = [[0] * 8, [63] * 8, [-64] * 8, [63, -64] * 4,
               list(range(-4, 4)), [32, -32] * 4]
    for index in range(8):
        for amplitude in (-64, 32, 63):
            samples = [0] * 8
            samples[index] = amplitude
            vectors.append(samples)
    vectors += [[rng.randrange(-64, 64) for _ in range(8)] for _ in range(100)]
    for index, samples in enumerate(vectors):
        dut.ena.value = index & 1
        await frame(dut, samples, rng, gaps=bool(index & 1))
    assert await cycle(dut) == (0, 0)


@cocotb.test()
async def test_reset_during_each_phase(dut):
    rng = random.Random(123)
    # Reset at every load, compute, and output position, then verify recovery.
    for elapsed in range(36):
        await reset(dut)
        for index in range(elapsed):
            await cycle(dut, (index * 17) & 127, 1)
        before = (int(dut.uo_out.value), int(dut.uio_out.value))
        dut.clk.value = 0
        dut.rst_n.value = 0
        await Timer(1, unit="us")
        assert (int(dut.uo_out.value), int(dut.uio_out.value)) == before, (
            "reset must not change registered outputs before a rising edge"
        )
        await reset(dut)
        await frame(dut, [32] * 8, rng, gaps=False)
        assert await cycle(dut) == (0, 0)
