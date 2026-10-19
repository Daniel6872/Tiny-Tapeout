<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project implements an 8-point radix-2 DIT FFT using one combinational
butterfly and an 8 x 14-bit register file. Each word holds signed 7-bit real
and imaginary components. Inputs are real, loaded in bit-reversed address order;
outputs are complex bins in natural order.

The butterfly uses 181/256 (0.70703125, eight fractional bits) for the diagonal
twiddles, implemented with shifts and adds, and divides results by two at
each stage, giving an approximately FFT/8 output. Both coefficient multiplication
and stage scaling round to nearest, with ties toward positive infinity. Results
saturate to [-64, 63].

| Stage | (Addr_A, Addr_B, twiddle index), in execution order |
|-------|--------------------------------------------------|
| 0 | (0,1,0), (2,3,0), (4,5,0), (6,7,0) |
| 1 | (0,2,0), (1,3,2), (4,6,0), (5,7,2) |
| 2 | (0,4,0), (1,5,1), (2,6,2), (3,7,3) |

The register file reads asynchronously and captures both butterfly results on
the rising edge, before the controller advances to the next address pair.

## How to test

Hold `rst_n` low across a rising clock edge to synchronously clear all state and
register-file words. During load, each rising edge with `uio_in[0]` high accepts
one signed sample from `ui_in[6:0]`; gaps are allowed. The eighth sample starts
12 compute cycles. Inputs are ignored during computation and output.

The following 16 cycles output X[0] through X[7], real byte then imaginary byte,
sign-extended to eight bits on `uo_out`. `uio_out[1]` is high for each valid byte;
all other bidirectional output bits are zero, and `uio_oe` is 0x02. Output data is
zero when invalid. The first output byte appears after the 13th rising edge
following acceptance of the eighth sample. A new frame can start on the edge
immediately after the last output byte. A frame takes 36 edges without load gaps.

Run `make` in the test directory for bit-exact FFT, pin-protocol, and synchronous
reset regressions, or `make butterfly` for the standalone arithmetic tests.
The fixed-point Python model is in [test/test.py](../test/test.py), using
[test/test_butterfly.py](../test/test_butterfly.py) for butterfly arithmetic.

## External hardware

A clock and a digital host capable of supplying samples and capturing output
bytes. No analog hardware is required.
