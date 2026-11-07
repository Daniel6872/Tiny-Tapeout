<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

An 8-point radix-2 DIT FFT on signed 7-bit real samples, computed with a single butterfly unit and an 8-word register file. Each frame takes exactly 36 clock cycles: 8 load cycles, 12 compute cycles (one butterfly per cycle across three stages), and 16 output cycles. Every stage halves its results, so the outputs are X[k]/8 and cannot overflow. Frames repeat back to back.

## How to test

Run the cocotb testbench in `test/` (`make -B`). On the chip:

1. While `uio[0]` (load_valid) is high, a signed 7-bit sample is taken from `ui[6:0]` on every rising clock edge (`ui[7]` is ignored). Present x[0]..x[7] on eight consecutive edges.
2. Wait 12 compute cycles (`uio[1:0]` both low; `uo` is 0).
3. While `uio[1]` (output_valid) is high, `uo` streams 16 bytes: for each bin X[0]..X[7], first `{0, real[6:0]}` then `{1, imag[6:0]}` (signed 7-bit values).

## External hardware

None required. Uses the standard Tiny Tapeout input switch bank (`ui_in`), bidirectional bank (`uio`, driven as outputs), and output LEDs (`uo_out`)
