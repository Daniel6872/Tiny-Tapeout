import itertools
import random

import cocotb
from cocotb.triggers import Timer


def butterfly_reference(a, b, twiddle, shift):
    """Model 45/64 twiddles and two nearest-rounding steps, ties toward +inf."""
    ar, ai = a
    br, bi = b

    def coefficient(value):
        return (45 * value + 32) // 64

    if twiddle == 0:
        tr, ti = br, bi
    elif twiddle == 1:
        tr, ti = coefficient(br + bi), coefficient(bi - br)
    elif twiddle == 2:
        tr, ti = bi, -br
    elif twiddle == 3:
        tr, ti = coefficient(bi - br), coefficient(-br - bi)
    else:
        raise ValueError("twiddle must be in 0..3")

    if shift not in range(4):
        raise ValueError("shift must be in 0..3")

    def scale(value):
        rounded = (value + (1 << (shift - 1))) // (1 << shift) if shift else value
        return max(-64, min(63, rounded))

    return (scale(ar + tr), scale(ai + ti)), (scale(ar - tr), scale(ai - ti))


def pack(value):
    real, imag = value
    return ((real & 127) << 7) | (imag & 127)


async def check_case(dut, a, b, twiddle, shift):
    dut.Read_A.value = pack(a)
    dut.Read_B.value = pack(b)
    dut.Twiddle_Index.value = twiddle
    dut.Shift_Ctrl.value = shift
    await Timer(1, unit="ns")
    expected_a, expected_b = butterfly_reference(a, b, twiddle, shift)
    actual = (int(dut.Write_A_prime.value), int(dut.Write_B_prime.value))
    expected = (pack(expected_a), pack(expected_b))
    assert actual == expected, (
        f"A={a}, B={b}, W={twiddle}, shift={shift}: "
        f"got packed {actual}, expected {expected} ({expected_a}, {expected_b})"
    )


@cocotb.test()
async def test_boundaries(dut):
    # Hand-calculated anchors catch sign, rounding, and saturation model errors.
    anchors = [
        ((0, 0), (32, 0), 1, 1, ((12, -11), (-11, 11))),
        ((0, 0), (-64, -64), 2, 0, ((-64, 63), (63, -64))),
        ((0, 0), (-64, -64), 3, 1, ((0, 45), (0, -45))),
        ((63, -64), (63, -64), 0, 0, ((63, -64), (0, 0))),
    ]
    for a, b, twiddle, shift, expected in anchors:
        assert butterfly_reference(a, b, twiddle, shift) == expected
        await check_case(dut, a, b, twiddle, shift)

    edges = (-64, -63, -1, 0, 1, 62, 63)
    for ar, ai, br, bi in itertools.product(edges, repeat=4):
        for twiddle, shift in itertools.product(range(4), repeat=2):
            await check_case(dut, (ar, ai), (br, bi), twiddle, shift)


@cocotb.test()
async def test_exhaustive_twiddles(dut):
    # All 16,384 complex B values, all twiddles, and all scaling modes.
    for br, bi in itertools.product(range(-64, 64), repeat=2):
        for twiddle, shift in itertools.product(range(4), repeat=2):
            await check_case(dut, (0, 0), (br, bi), twiddle, shift)


@cocotb.test()
async def test_random_butterflies(dut):
    rng = random.Random(0x8FF7)
    for _ in range(20000):
        a = (rng.randrange(-64, 64), rng.randrange(-64, 64))
        b = (rng.randrange(-64, 64), rng.randrange(-64, 64))
        await check_case(dut, a, b, rng.randrange(4), rng.randrange(4))
