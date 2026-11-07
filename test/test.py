"""Pin-level cocotb tests for the 8-point fixed-point FFT (tt_um_example).

Pin protocol (fft_io_wrapper):
  * uio_out[0] is high during LOAD. A sample is taken from ui_in[6:0] at every
    rising edge in LOAD; ui_in[7] and all uio_in pins are ignored.
  * After 8 samples there are 12 compute cycles, then 16 OUTPUT cycles with
    uio_out[1] high. uo_out is {0, real[6:0]} then {1, imag[6:0]} for X[0]..X[7],
    and 0 at all other times. uio_oe is 0x03.
  * Frames repeat every 36 cycles.

Pins are checked after every rising edge against a cycle-accurate model. On RTL,
the FSM controller's outputs are also checked before every edge against a
schedule derived from the radix-2 DIT definition. With GATES=yes the same tests
run on the gate-level netlist with the RTL-only internal checks skipped.
"""

import cmath
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

GATE_LEVEL = os.getenv("GATES") == "yes"

LOAD_ORDER = [0, 4, 2, 6, 1, 5, 3, 7]  # Register address for sample x[n]
LOAD_CYCLES = 8
COMPUTE_CYCLES = 12
OUTPUT_BYTES = 16
FRAME_EDGES = LOAD_CYCLES + COMPUTE_CYCLES + OUTPUT_BYTES
UIO_LOAD = 0b01
UIO_OUTPUT = 0b10


def _dit_schedule():
    """(addr_a, addr_b, twiddle) per butterfly: stages of span 2, 4 and 8."""
    schedule = []
    for stage in range(3):
        half = 1 << stage
        span = 2 * half
        for base in range(0, 8, span):
            for j in range(half):
                schedule.append((base + j, base + j + half, j * 8 // span))
    return schedule


COMPUTE_SCHEDULE = _dit_schedule()


def _sqrt_half(value):
    # 1/sqrt(2) ~= 181/256, rounded to nearest with ties toward +infinity.
    return (181 * value + 128) >> 8


def scale_and_saturate(value, shift):
    # Round to nearest with ties toward +infinity, then saturate to signed 7-bit.
    return max(-64, min(63, (value + (1 << (shift - 1) if shift else 0)) >> shift))


def butterfly(a, b, twiddle, shift=1):
    """Apply one rounded, saturated butterfly for W = exp(-j*2*pi*twiddle/8)."""
    ar, ai = a
    br, bi = b
    if twiddle == 0:
        tr, ti = br, bi
    elif twiddle == 1:
        tr, ti = _sqrt_half(br + bi), _sqrt_half(bi - br)
    elif twiddle == 2:
        tr, ti = bi, -br
    elif twiddle == 3:
        tr, ti = _sqrt_half(bi - br), _sqrt_half(-br - bi)
    else:
        raise ValueError("twiddle must be 0..3")
    return (
        (scale_and_saturate(ar + tr, shift), scale_and_saturate(ai + ti, shift)),
        (scale_and_saturate(ar - tr, shift), scale_and_saturate(ai - ti, shift)),
    )


def fft_reference(samples):
    """Bit-exact model: bit-reversed load, then three radix-2 DIT stages."""
    if len(samples) != 8 or not all(-64 <= x <= 63 for x in samples):
        raise ValueError("expected eight signed 7-bit samples")
    data = [(0, 0)] * 8
    for n, x in enumerate(samples):
        data[LOAD_ORDER[n]] = (x, 0)
    for a, b, twiddle in COMPUTE_SCHEDULE:
        data[a], data[b] = butterfly(data[a], data[b], twiddle)
    return data


def expected_bytes(samples):
    """X[0]..X[7] as {0, real[6:0]} then {1, imag[6:0]}."""
    out = []
    for real, imag in fft_reference(samples):
        out += [real & 0x7F, 0x80 | (imag & 0x7F)]
    return out


def decode_byte(byte):
    """Recover the signed 7-bit value from a tagged output byte."""
    return ((byte & 0x7F) ^ 0x40) - 0x40


class FrameModel:
    """Cycle-accurate model of the FSM state and the resulting pins."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = "LOAD"
        self.samples = []
        self.count = 0
        self.bytes = []

    def pins(self):
        """(uo_out, uio_out) for the current state; uo_out is combinational."""
        if self.phase == "LOAD":
            return 0, UIO_LOAD
        if self.phase == "COMPUTE":
            return 0, 0
        return self.bytes[self.count], UIO_OUTPUT

    def controls(self):
        """FSM outputs the datapath uses at the next rising edge.

        Signals left out are don't-cares in that phase.
        """
        if self.phase == "LOAD":
            return {"phase_state": 0, "we_load": 1, "we_math": 0, "is_valid_output": 0,
                    "addr_a": LOAD_ORDER[len(self.samples)]}
        if self.phase == "COMPUTE":
            addr_a, addr_b, twiddle = COMPUTE_SCHEDULE[self.count]
            return {"phase_state": 1, "we_load": 0, "we_math": 1, "is_valid_output": 0,
                    "addr_a": addr_a, "addr_b": addr_b, "twiddle_index": twiddle,
                    "shift_ctrl": 1}
        return {"phase_state": 2, "we_load": 0, "we_math": 0, "is_valid_output": 1,
                "addr_a": self.count // 2, "output_imag": self.count % 2}

    def edge(self, rst_n, sample):
        if not rst_n:
            self.reset()
        elif self.phase == "LOAD":
            self.samples.append(sample)
            if len(self.samples) == LOAD_CYCLES:
                self.bytes = expected_bytes(self.samples)
                self.phase = "COMPUTE"
                self.count = 0
        elif self.phase == "COMPUTE":
            self.count += 1
            if self.count == COMPUTE_CYCLES:
                self.phase = "OUTPUT"
                self.count = 0
        else:
            self.count += 1
            if self.count == OUTPUT_BYTES:
                self.reset()
        return self.pins()


class Harness:
    def __init__(self, dut, seed):
        self.dut = dut
        self.rng = random.Random(seed)
        self.model = FrameModel()
        self.edge = 0

    async def start(self):
        self.dut.ena.value = 1
        self.dut.rst_n.value = 0
        self.dut.ui_in.value = 0
        self.dut.uio_in.value = 0
        Clock(self.dut.clk, 10, unit="us").start(start_high=False)

    def read_pins(self):
        uo_out = int(self.dut.uo_out.value)
        uio_out = int(self.dut.uio_out.value)
        assert int(self.dut.uio_oe.value) == 0b0000_0011, "uio_oe must be 0x03"
        assert uio_out & 0xFC == 0, f"uio_out[7:2] must be 0: {uio_out:#04x}"
        return uo_out, uio_out

    async def tick(self, sample=None, rst_n=True, randomize_ignored_pins=True):
        """Drive one rising edge; check FSM controls before it and pins after it."""
        rng = self.rng
        if sample is None:
            sample = rng.randrange(-64, 64) if randomize_ignored_pins else 0
        ignored_bit = rng.getrandbits(1) if randomize_ignored_pins else 0
        ignored_uio = rng.getrandbits(8) if randomize_ignored_pins else 0
        self.dut.ui_in.value = (ignored_bit << 7) | (sample & 0x7F)
        self.dut.uio_in.value = ignored_uio
        self.dut.rst_n.value = int(rst_n)
        if rst_n and not GATE_LEVEL:
            await Timer(1, unit="ns")  # Let combinational FSM outputs settle
            check_controls(self.dut, self.model, self.edge)
        await RisingEdge(self.dut.clk)
        await FallingEdge(self.dut.clk)
        expected = self.model.edge(rst_n, decode_byte(sample & 0x7F))
        actual = self.read_pins()
        assert actual == expected, (
            f"edge {self.edge}: got (uo_out={actual[0]:#04x}, uio_out={actual[1]:#04b}), "
            f"expected (uo_out={expected[0]:#04x}, uio_out={expected[1]:#04b})"
        )
        self.edge += 1
        return actual

    async def reset(self, cycles=2):
        for _ in range(cycles):
            # Reset forces LOAD, so uio_out[0] is already high.
            assert await self.tick(rst_n=False) == (0, UIO_LOAD)
        check_reset_state(self.dut)

    async def run_frame(self, samples, randomize_ignored_pins=True):
        """Send one frame, run it to completion, and check bytes and timing."""
        assert self.model.phase == "LOAD" and not self.model.samples
        for index, x in enumerate(samples):
            pins = await self.tick(x, randomize_ignored_pins=randomize_ignored_pins)
            assert pins == ((0, UIO_LOAD) if index < 7 else (0, 0))
        last_sample_edge = self.edge - 1

        # ui_in carries random data while busy; it must be ignored.
        received = []
        valid_edges = []
        for _ in range(COMPUTE_CYCLES + OUTPUT_BYTES):
            byte, uio = await self.tick(randomize_ignored_pins=randomize_ignored_pins)
            if uio & UIO_OUTPUT:
                received.append(byte)
                valid_edges.append(self.edge - 1)
            else:
                assert byte == 0, "uo_out must be 0 outside OUTPUT"

        first = last_sample_edge + COMPUTE_CYCLES
        assert valid_edges == list(range(first, first + OUTPUT_BYTES)), (
            f"valid bytes after edges {valid_edges}, expected {first}..{first + 15}"
        )
        assert received == expected_bytes(samples), (
            f"samples={samples}: got {received}, expected {expected_bytes(samples)}"
        )
        assert self.model.phase == "LOAD", "must return to LOAD after 16 bytes"
        return received


def check_controls(dut, model, edge):
    """RTL only: compare FSM outputs with the expected schedule."""
    top = dut.user_project
    fsm = top.controller_inst
    expected = model.controls()
    actual = {name: int(getattr(fsm, name).value) for name in expected}
    assert actual == expected, f"edge {edge}: FSM outputs {actual}, expected {expected}"
    if model.phase == "OUTPUT":
        # fft_io_wrapper selects real/imaginary itself; it must agree with the FSM.
        selector = int(top.io_wrapper_inst.stream_cycle.value) & 1
        assert selector == expected["output_imag"], (
            f"edge {edge}: wrapper selects {'imag' if selector else 'real'} for byte "
            f"{model.count}"
        )


def check_reset_state(dut):
    """RTL only: reset must clear the controller and every register-file word."""
    if GATE_LEVEL:
        return
    top = dut.user_project
    assert int(top.controller_inst.phase_state.value) == 0
    assert int(top.controller_inst.step.value) == 0
    assert int(top.io_wrapper_inst.stream_cycle.value) == 0
    memory = top.datapath_inst.reg_file_inst.mem
    words = [int(memory[i].value) for i in range(8)]
    assert words == [0] * 8, f"register file not cleared: {words}"


def edge_vectors(rng):
    vectors = [[0] * 8, [63] * 8, [-64] * 8, [63, -64] * 4, [-64, 63] * 4,
               list(range(-4, 4)), [32, -32] * 4]
    for n in range(8):
        for amplitude in (-64, -1, 1, 32, 63):
            impulse = [0] * 8
            impulse[n] = amplitude
            vectors.append(impulse)
    vectors += [[rng.choice((-64, 63)) for _ in range(8)] for _ in range(16)]
    return vectors


def ideal_dft8(samples):
    return [sum(x * cmath.exp(-2j * cmath.pi * k * n / 8) for n, x in enumerate(samples)) / 8
            for k in range(8)]


@cocotb.test()
async def test_reference_matches_ideal_fft(dut):
    """The bit-exact model must track an ideal DFT/8 within the rounding bound."""
    # Per stage: halving rounds by <= 0.5 LSB, W1/W3 products by <= 0.25 LSB
    # after halving, and errors grow by at most (1 + sqrt(2))/2. That gives
    # 0.5, 1.0, then ~1.97 LSB after three stages.
    rng = random.Random(0x1D7)
    vectors = edge_vectors(rng) + [[rng.randrange(-64, 64) for _ in range(8)]
                                   for _ in range(5000)]
    for samples in vectors:
        for k, (got, ideal) in enumerate(zip(fft_reference(samples), ideal_dft8(samples))):
            error = max(abs(got[0] - ideal.real), abs(got[1] - ideal.imag))
            assert error <= 2.0, f"X[{k}] of {samples}: {got} vs {ideal:.3f}"
    dut._log.info("Fixed-point model is within 2 LSB of DFT/8 for %d vectors", len(vectors))


@cocotb.test()
async def test_manual_single_frame(dut):
    """Run one varied signed-input frame and verify real and imaginary bins."""
    harness = Harness(dut, 0x298A)
    await harness.start()
    await harness.reset()

    samples = [-28, 40, -14, 43, 59, 15, 24, -8]
    expected_bins = [
        (17, 0), (-12, -2), (3, -2), (-8, -11),
        (-6, 0), (-8, 12), (3, 3), (-12, 3),
    ]
    assert fft_reference(samples) == expected_bins
    expected = expected_bytes(samples)
    dut._log.info("Input samples: %s", samples)
    dut._log.info("ui_in bus bytes (ignored bit 7 held low): %s",
                  [f"0x{sample & 0x7F:02X}" for sample in samples])
    dut._log.info("Expected (real, imag) bins: %s", expected_bins)
    assert await harness.run_frame(samples, randomize_ignored_pins=False) == expected


@cocotb.test()
async def test_hand_checked_frames(dut):
    """Known inputs with hand-calculated outputs, sent as back-to-back frames."""
    harness = Harness(dut, 1)
    await harness.start()
    await harness.reset()
    zero_bin = [0x00, 0x80]
    cases = [
        ([32] * 8, [32, 0x80] + zero_bin * 7),                      # DC: X[0] = 256/8
        ([32, 0, 0, 0, 0, 0, 0, 0], [4, 0x80] * 8),                 # Impulse: flat
        ([32, -32] * 4, zero_bin * 4 + [32, 0x80] + zero_bin * 3),  # Nyquist: X[4]
        ([-64] * 8, [0x40, 0x80] + zero_bin * 7),                   # X[0] = -64
        ([0] * 8, zero_bin * 8),
    ]
    for samples, expected in cases:
        assert expected_bytes(samples) == expected, f"model disagrees for {samples}"
        assert await harness.run_frame(samples) == expected
    assert decode_byte(0x40) == -64 and decode_byte(0xC0) == -64


@cocotb.test()
async def test_edge_and_random_frames(dut):
    """Extremes and random data in back-to-back frames, with random ignored pins."""
    harness = Harness(dut, 2)
    await harness.start()
    await harness.reset()
    rng = random.Random(0xFF7)
    vectors = edge_vectors(rng) + [[rng.randrange(-64, 64) for _ in range(8)]
                                   for _ in range(150)]
    for samples in vectors:
        received = await harness.run_frame(samples)
        decoded = [(decode_byte(received[2 * k]), decode_byte(received[2 * k + 1]))
                   for k in range(8)]
        assert decoded == fft_reference(samples)


@cocotb.test()
async def test_continuous_streaming_timing(dut):
    """Frames repeat every 36 edges with the exact uio_out pattern."""
    harness = Harness(dut, 3)
    await harness.start()
    await harness.reset()
    rng = random.Random(0x5EED)
    frames = 6
    presented = [rng.randrange(-64, 64) for _ in range(frames * FRAME_EDGES)]
    pins = [await harness.tick(sample) for sample in presented]

    # After edge i of a frame: LOAD while fewer than 8 samples are in, then
    # COMPUTE for 12 edges, OUTPUT for 16 edges, and LOAD again.
    pattern = ([UIO_LOAD] * (LOAD_CYCLES - 1) + [0] * COMPUTE_CYCLES
               + [UIO_OUTPUT] * OUTPUT_BYTES + [UIO_LOAD])
    for frame in range(frames):
        base = frame * FRAME_EDGES
        window = pins[base:base + FRAME_EDGES]
        assert [uio for _, uio in window] == pattern, f"frame {frame} uio_out timing"
        accepted = presented[base:base + LOAD_CYCLES]
        start = LOAD_CYCLES - 1 + COMPUTE_CYCLES
        assert [b for b, _ in window[start:start + OUTPUT_BYTES]] == expected_bytes(accepted)


@cocotb.test()
async def test_reset_at_every_frame_position(dut):
    """Synchronous reset anywhere in a frame aborts it; the next frame is correct."""
    harness = Harness(dut, 4)
    await harness.start()
    await harness.reset()
    rng = random.Random(0x12E5)
    for offset in range(FRAME_EDGES + 1):
        for _ in range(offset):
            await harness.tick()
        await harness.reset(cycles=1 + offset % 3)
        await harness.run_frame([rng.randrange(-64, 64) for _ in range(8)])


async def unit_defaults(dut):
    """Initialize the independent RTL-module instances in tb.v."""
    dut.clk.value = 0
    dut.rst_n.value = 0
    dut.ena.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0

    for name in (
        "unit_rf_rst_n", "unit_rf_we_load", "unit_rf_we_math",
        "unit_dp_rst_n", "unit_dp_we_load", "unit_dp_we_math",
        "unit_io_rst_n", "unit_ctrl_rst_n", "unit_ctrl_is_valid_input",
    ):
        getattr(dut, name).value = 0
    for name in (
        "unit_rf_addr_a", "unit_rf_addr_b", "unit_rf_d_load",
        "unit_rf_write_a", "unit_rf_write_b", "unit_bf_read_a",
        "unit_bf_read_b", "unit_bf_twiddle", "unit_bf_shift",
        "unit_dp_addr_a", "unit_dp_addr_b", "unit_dp_d_load",
        "unit_dp_twiddle", "unit_dp_shift", "unit_io_ui_in",
        "unit_io_phase", "unit_io_d_out",
    ):
        getattr(dut, name).value = 0
    await Timer(1, unit="ns")


async def unit_edge(dut):
    """Generate one controlled clock edge for the standalone unit instances."""
    dut.clk.value = 0
    await Timer(5, unit="ns")
    dut.clk.value = 1
    await Timer(5, unit="ns")
    dut.clk.value = 0
    await Timer(1, unit="ns")


def complex_word(real, imag):
    return ((real & 0x7F) << 7) | (imag & 0x7F)


@cocotb.test()
async def test_reg_file_module(dut):
    """Check reset, read addressing, load priority, dual math writes and hold."""
    if GATE_LEVEL:
        return
    await unit_defaults(dut)
    rf = dut.unit_reg_file
    rf.rst_n.value = 0
    await unit_edge(dut)
    rf.rst_n.value = 1

    for address in range(8):
        rf.addr_a.value = address
        await Timer(1, unit="ns")
        assert int(rf.read_a.value) == 0, f"reset did not clear word {address}"

    for address in range(8):
        rf.addr_a.value = address
        rf.we_load.value = 1
        rf.d_load.value = (address * 137 + 19) & 0x3FFF
        await unit_edge(dut)
        assert int(rf.read_a.value) == (address * 137 + 19) & 0x3FFF
        assert int(rf.d_out.value) == int(rf.read_a.value)
    rf.we_load.value = 0

    rf.addr_a.value = 2
    rf.addr_b.value = 3
    await Timer(1, unit="ns")
    rf.d_load.value = 0x1234
    rf.write_a_prime.value = 0x0111
    rf.write_b_prime.value = 0x0222
    rf.we_load.value = 1
    rf.we_math.value = 1
    await unit_edge(dut)
    rf.we_load.value = 0
    rf.we_math.value = 0
    assert int(rf.read_a.value) == 0x1234, "load must take priority over math"
    rf.addr_a.value = 3
    await Timer(1, unit="ns")
    assert int(rf.read_a.value) == 3 * 137 + 19, "priority must suppress both math writes"

    rf.addr_a.value = 2
    rf.we_math.value = 1
    await unit_edge(dut)
    rf.we_math.value = 0
    assert int(rf.read_a.value) == 0x0111
    rf.addr_a.value = 3
    await Timer(1, unit="ns")
    assert int(rf.read_a.value) == 0x0222

    rf.we_load.value = 0
    rf.we_math.value = 0
    rf.addr_a.value = 2
    await unit_edge(dut)
    assert int(rf.read_a.value) == 0x0111, "registers must hold when both enables are low"


@cocotb.test()
async def test_butterfly_module(dut):
    """Exhaustively vary twiddles and shifts over boundary and random operands."""
    if GATE_LEVEL:
        return
    await unit_defaults(dut)
    bf = dut.unit_fft_butterfly
    rng = random.Random(0xB0773)
    operands = [
        ((-64, -64), (-64, -64)),
        ((63, 63), (63, 63)),
        ((-64, 63), (-64, 63)),
        ((63, -64), (63, -64)),
    ]
    operands.extend(
        (
            (tuple(rng.randrange(-64, 64) for _ in range(2)),
             tuple(rng.randrange(-64, 64) for _ in range(2)))
        )
        for _ in range(256)
    )
    for a, b in operands:
        for twiddle in range(4):
            for shift in range(4):
                bf.read_a.value = complex_word(*a)
                bf.read_b.value = complex_word(*b)
                bf.twiddle_index.value = twiddle
                bf.shift_ctrl.value = shift
                await Timer(1, unit="ns")
                expected_a, expected_b = butterfly(a, b, twiddle, shift)
                assert int(bf.write_a_prime.value) == complex_word(*expected_a), (
                    f"A' mismatch for A={a}, B={b}, twiddle={twiddle}, shift={shift}"
                )
                assert int(bf.write_b_prime.value) == complex_word(*expected_b), (
                    f"B' mismatch for A={a}, B={b}, twiddle={twiddle}, shift={shift}"
                )


@cocotb.test()
async def test_datapath_module(dut):
    """Check register-file loading and butterfly writeback through the datapath."""
    if GATE_LEVEL:
        return
    await unit_defaults(dut)
    dp = dut.unit_fft_datapath
    dp.rst_n.value = 0
    await unit_edge(dut)
    dp.rst_n.value = 1

    a = (37, -22)
    b = (-41, 58)
    dp.we_load.value = 1
    dp.addr_a.value = 0
    dp.d_load.value = complex_word(*a)
    await unit_edge(dut)
    dp.addr_a.value = 1
    dp.d_load.value = complex_word(*b)
    await unit_edge(dut)
    dp.we_load.value = 0

    dp.addr_a.value = 0
    dp.addr_b.value = 1
    dp.twiddle_index.value = 3
    dp.shift_ctrl.value = 1
    await Timer(1, unit="ns")
    expected_a, expected_b = butterfly(a, b, 3)
    dp.we_math.value = 1
    await unit_edge(dut)
    dp.we_math.value = 0
    assert int(dp.d_out.value) == complex_word(*expected_a)
    dp.addr_a.value = 1
    await Timer(1, unit="ns")
    assert int(dp.d_out.value) == complex_word(*expected_b)


@cocotb.test()
async def test_io_wrapper_module(dut):
    """Check pin mapping, phase-valid outputs, byte order, and stream counter reset."""
    if GATE_LEVEL:
        return
    await unit_defaults(dut)
    io = dut.unit_fft_io_wrapper
    io.rst_n.value = 0
    io.Phase_State.value = 2
    io.ui_in.value = 0xD5
    io.D_out.value = complex_word(-37, 46)
    await Timer(1, unit="ns")
    assert int(io.uio_out.value) == 0b10
    assert int(io.uio_oe.value) == 0x03
    assert int(io.uo_out.value) == 0x5B, "reset stream counter selects real output"
    assert int(io.D_load.value) == ((0xD5 & 0x7F) << 7)
    await unit_edge(dut)
    io.rst_n.value = 1

    io.Phase_State.value = 0
    await Timer(1, unit="ns")
    assert int(io.uio_out.value) == 0b01
    assert int(io.uo_out.value) == 0
    io.Phase_State.value = 1
    await Timer(1, unit="ns")
    assert int(io.uio_out.value) == 0
    assert int(io.uo_out.value) == 0

    io.Phase_State.value = 2
    for index in range(16):
        await Timer(1, unit="ns")
        expected = (0x80 | (46 & 0x7F)) if index & 1 else (-37 & 0x7F)
        assert int(io.uo_out.value) == expected, f"output byte {index}"
        assert int(io.uio_out.value) == 0b10
        await unit_edge(dut)
    assert int(io.uo_out.value) == (0x80 | (46 & 0x7F)), "counter must hold at byte 15"

    io.rst_n.value = 0
    await Timer(1, unit="ns")
    assert int(io.uo_out.value) == ((-37) & 0x7F), "asynchronous reset restarts at real"


def assert_controller_outputs(controller, expected, message):
    actual = {
        "phase": int(controller.phase_state.value),
        "we_load": int(controller.we_load.value),
        "we_math": int(controller.we_math.value),
        "addr_a": int(controller.addr_a.value),
        "addr_b": int(controller.addr_b.value),
        "twiddle": int(controller.twiddle_index.value),
        "shift": int(controller.shift_ctrl.value),
        "valid_output": int(controller.is_valid_output.value),
        "imag": int(controller.output_imag.value),
    }
    assert actual == expected, f"{message}: got {actual}, expected {expected}"


@cocotb.test()
async def test_controller_module(dut):
    """Check load stalls, every butterfly address/twiddle, and output sequencing."""
    if GATE_LEVEL:
        return
    await unit_defaults(dut)
    ctrl = dut.unit_fft_controller
    ctrl.rst_n.value = 0
    await unit_edge(dut)
    ctrl.rst_n.value = 1
    ctrl.is_valid_input.value = 0
    await Timer(1, unit="ns")
    assert_controller_outputs(
        ctrl, {"phase": 0, "we_load": 0, "we_math": 0, "addr_a": 0,
               "addr_b": 0, "twiddle": 0, "shift": 1, "valid_output": 0, "imag": 0},
        "load stall",
    )
    await unit_edge(dut)

    ctrl.is_valid_input.value = 1
    for address in LOAD_ORDER:
        await Timer(1, unit="ns")
        assert int(ctrl.phase_state.value) == 0
        assert int(ctrl.we_load.value) == 1
        assert int(ctrl.we_math.value) == 0
        assert int(ctrl.addr_a.value) == address
        await unit_edge(dut)

    for index, (addr_a, addr_b, twiddle) in enumerate(COMPUTE_SCHEDULE):
        await Timer(1, unit="ns")
        assert_controller_outputs(
            ctrl, {"phase": 1, "we_load": 0, "we_math": 1, "addr_a": addr_a,
                   "addr_b": addr_b, "twiddle": twiddle, "shift": 1,
                   "valid_output": 0, "imag": index & 1},
            f"compute step {index}",
        )
        await unit_edge(dut)

    ctrl.is_valid_input.value = 0
    for index in range(OUTPUT_BYTES):
        await Timer(1, unit="ns")
        assert_controller_outputs(
            ctrl, {"phase": 2, "we_load": 0, "we_math": 0, "addr_a": index // 2,
                   "addr_b": 0, "twiddle": 0, "shift": 1,
                   "valid_output": 1, "imag": index & 1},
            f"output step {index}",
        )
        await unit_edge(dut)
    await Timer(1, unit="ns")
    assert int(ctrl.phase_state.value) == 0, "controller must return to LOAD"
