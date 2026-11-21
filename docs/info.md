<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This project computes an 8-point radix-2 decimation-in-time (DIT) FFT on signed 7-bit real samples using one butterfly unit and an 8-word register file. Samples use 7-bit two's-complement encoding, so the input range is -64 to 63.

Each frame takes 36 clock cycles: 8 cycles to load the samples, 12 cycles to compute the three FFT stages (one butterfly per cycle), and 16 cycles to output the results. Each stage divides its butterfly sums and differences by two, rounds to nearest with ties toward positive infinity, and saturates to the signed 7-bit range. The diagonal twiddle factors approximate 1/sqrt(2) using 181/256. Consequently, the outputs approximate X[k]/8 and may differ slightly from an ideal, unquantized FFT.

## How to test

Run the cocotb testbench in `test/` (`make -B`). On the chip:

1. While `uio[0]` (`load_valid`) is high, the design samples `ui[6:0]` on each rising clock edge; `ui[7]` is ignored. Present x[0] through x[7] on eight consecutive edges.
2. The design computes for 12 cycles. During this time, `uio[1:0]` are low and `uo` is zero.
3. While `uio[1]` (`output_valid`) is high, `uo` emits 16 bytes: for each bin X[0] through X[7], the real component comes first and the imaginary component second. In each byte, `uo[6:0]` is a signed 7-bit two's-complement value, and `uo[7]` is 0 for real or 1 for imaginary.
4. `uio[2]` (`overflow`) is high only during the 16 output cycles, and only if any butterfly result saturated during that frame. It is low during loading, computation, and between frames. `uio[7:3]` are unused.

## External hardware

None required. Uses the standard Tiny Tapeout input switch bank (`ui_in`), bidirectional bank (`uio`, driven as outputs), and output LEDs (`uo_out`)
