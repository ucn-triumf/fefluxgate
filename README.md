# fefluxgate

A [MIDAS](https://daq00.triumf.ca/MidasWiki) frontend for the orange TRIUMF fluxgate
magnetometer DAQ box.

## Overview

The frontend connects to the DAQ box over ZMQ, continuously polls the streamed
magnetometer data in a background thread, and publishes it into MIDAS as a periodic
equipment. Every second it writes the latest reading of all 16 magnetometers into a
single `FG00` bank. Acquisition parameters (rate, filtering, which magnetometers to
read, etc.) are set live through the ODB.

## Files

| File | Purpose |
| --- | --- |
| `fefluxgate.py` | MIDAS frontend; defines the `FluxGate` periodic equipment and ODB settings. |
| `FluxgateBox.py` | ZMQ interface to the DAQ box (background polling + register read/write). |
| `start_fefluxgate.sh` | Environment setup and launch script. |

## Requirements

- MIDAS with its Python bindings (`midas.frontend`, `midas.event`)
- `pyzmq`
- `numpy`

## Running

```sh
./start_fefluxgate.sh
```

The script sets `MIDASSYS`/`PYTHONPATH`, activates a Python virtual environment, and
runs `python3 fefluxgate.py`. Paths in the script are hard-coded for the UCN DAQ host —
adjust them for your environment. With MIDAS already configured you can also run
`python3 fefluxgate.py` directly.

## Configuration

Settings live under `/Equipment/FluxGate/Settings` in the ODB and are applied
immediately when changed (invalid values are rejected with a message and reset to a
safe default).

| Setting | Default | Allowed values | Description |
| --- | --- | --- | --- |
| `host_ip` | `142.90.151.5` | — | IP address of the DAQ box. |
| `rate` | `0` | `0`–`22` | ADC sample rate; `0` is fastest. |
| `settle_delay` | `0` | `0`–`7` | Settling delay. |
| `enhanced_filter` | `0` | `0, 2, 3, 5, 6` | Enhanced filter mode (`0` disables it). |
| `use_sinc3` | `False` | `True`/`False` | Use sinc3 filter (ignored when the enhanced filter is on). |
| `use_magnetometer` | `0xffff` | `0x0000`–`0xFFFF` | Bitmask of enabled magnetometers (bit 0 = Mag 1 … bit 15 = Mag 16). |
| `poll_ms` | `10` | — | Interval between ZMQ polls of the box, in milliseconds. |

## Data format

The `FG00` bank (event id 2323, `SYSTEM` buffer) contains **48 floats**: magnetometers
1–16, each as an (x, y, z) triple. Values are converted from the 24-bit bipolar ADC
codes to volts by `raw_to_voltage` in `FluxgateBox.py` (internal 2.5 V reference).
