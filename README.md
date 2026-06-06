# nRF52840 9-DoF Tracker

Firmware for a wireless full-body VR tracker for a **SuperMini nRF52840** board
and ICM-45686 + QMC6309 sensors. The trackers use 2.4 GHz radio for transmiting
its orientation to the receiver dongle.

The whole project is made up of three repositories:

| Component | Repo | Role |
|-----------|------|------|
| **Transmitter (this repo)** | [nrf52840_9dof_transmitter](https://github.com/Glebhl/nrf52840_9dof_transmitter) | Worn tracker(s): sense → fuse → transmit |
| Receiver / USB dongle | [nrf52840_9dof_receiver](https://github.com/Glebhl/nrf52840_9dof_receiver) | Listens to all trackers, bridges packets to USB serial |
| SteamVR driver | [nrf52840_steamvr_driver](https://github.com/Glebhl/nrf52840_steamvr_driver) | OpenVR driver that exposes the trackers to SteamVR |

```
  ┌──────────────┐   2.4 GHz radio    ┌────────────┐   USB serial    ┌──────────────┐
  │  Tracker(s)  │  ───────────────>  │  Receiver  │  ────────────>  │ SteamVR      │
  │  (this repo) │    many → one      │  dongle    │  binary frames  │ driver (DLL) │
  └──────────────┘                    └────────────┘                 └──────────────┘
```

> ⚠️ **Status: work in progress.** Not all functionality is implemented and the
> system is not yet fully working end to end.

## Hardware

- **MCU board:** SuperMini nRF52840.
- **IMU:** TDK InvenSense **ICM-45686** (3-axis accelerometer + 3-axis gyroscope).
- **Magnetometer:** **QMC6309** (3-axis).
- Both sensors must sit on the common I²C bus (default `SDA = P0.24`, `SCL = P0.22`,
  400 kHz).
- A user **button** (default `P0.06`, active-low).

The sensor module can optionally be powered straight from GPIO pins
("stacked power", default `P0.17` high / `P0.20` low) instead of a dedicated rail — see
`ENABLE_STACKED_POWER` in [`src/config.h`](src/config.h).

### Button & power

- **Single click** → run the calibration sequence (gyro bias, then magnetometer).
- **Long press** → enter **deep sleep** (~1 µA). The same button wakes the chip.


## Building & flashing

This is a [PlatformIO](https://platformio.org/) project.

```bash
# Build
pio run

# Put the controller into DFU mode and upload the code
python gen_uf2.py

```

### Configuration

Almost everything is tunable in [`src/config.h`](src/config.h): I²C pins, sensor
full-scale ranges, fusion gains (`Kp`/`Ki`), calibration sample counts, button
timings, telemetry rate, and which fields are transmitted.
