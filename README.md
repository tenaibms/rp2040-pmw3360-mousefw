# RP2040+PMW3360 FW

Firmware compatible with [this pcb](https://github.com/jfedor2/rp2040-pmw3360).

# Build Instructions

## MinGW
This assumes a working pico environment.
```bash
mkdir build
cmake -G"MinGW Makefiles" ..
make -j16
```
