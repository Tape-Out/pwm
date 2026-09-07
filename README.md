# pwm

Pulse-width modulator with configurable channels and dead time.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`loom`](https://github.com/Tape-Out/loom). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Planned. What sits in this repository today is the retired picorv32-era Verilog,
kept for provenance; the Bluespec rewrite has not landed yet.

## Notes

开始采用：
- RO - READ ONLY
- WO - WRITE ONLY
- RW - READ WRITE
- TP - TEMP WIRE OR REG - DEFAULT - 默认不加

## License

Mulan PSL v2.
