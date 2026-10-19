# Sample testbench for a Tiny Tapeout project

This is a sample testbench for a Tiny Tapeout project. It uses [cocotb](https://docs.cocotb.org/en/stable/) to drive the DUT and check the outputs.
See below to get started or for more information, check the [website](https://tinytapeout.com/hdl/testing/).

## Setting up

1. Edit [Makefile](Makefile) and modify `PROJECT_SOURCES` to point to your Verilog files.
2. Edit [tb.v](tb.v) and replace `tt_um_example` with your module name.

## How to run

To run the RTL simulation:

```sh
make -B
```

To test the standalone FFT butterfly with Icarus in Verilog-2005 mode:

```sh
make butterfly
```

This runs three cocotb tests against the integer reference function in
[test_butterfly.py](test_butterfly.py): signed boundary combinations, every
complex B input with all four twiddles and scaling settings, and 20,000 seeded
random butterflies. The reference uses 45/64 twiddles, nearest rounding with ties
toward positive infinity, and saturation to [-64, 63]. Results are written to
`results_butterfly.xml`. The existing GitHub Actions test workflow runs this target
in addition to the top-level tests, then synthesizes the standalone butterfly
with Yosys and checks the resulting netlist.

The butterfly target tests RTL only. The current top-level wrapper still contains
the counter, so the GDS workflow does not yet synthesize or measure the butterfly.

To run gatelevel simulation, first harden your project and copy `../runs/wokwi/results/final/verilog/gl/{your_module_name}.v` to `gate_level_netlist.v`.

Then run:

```sh
make -B GATES=yes
```

If you wish to save the waveform in VCD format instead of FST format, edit tb.v to use `$dumpfile("tb.vcd");` and then run:

```sh
make -B FST=
```

This will generate `tb.vcd` instead of `tb.fst`.

## How to view the waveform file

Using GTKWave

```sh
gtkwave tb.fst tb.gtkw
```

Using Surfer

```sh
surfer tb.fst
```
