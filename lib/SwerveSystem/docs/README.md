# SingleModuleTest Documentation

This folder contains the documentation for the `SingleModuleTest` swerve subsystem library used by the 10583 Stampede MiniFRC 2026 robot code.

## Contents

- [SingleModuleTest Manual](SingleModuleTest.md) - complete overview, wiring notes, API reference, and usage example.

## Current Library Behavior

- Encoder channels are read from an external Adafruit seesaw breakout over GPIO, not from direct MCU encoder pins.
- The encoder interrupt line is assumed to be wired to ESP GPIO 9.
- Module encoder pins are fixed in the library in this order:
  - Encoder 0: A=8, B=9
  - Encoder 1: A=10, B=11
  - Encoder 2: A=2, B=3
  - Encoder 3: A=4, B=5
- `QuicEncoder` keeps the same quadrature decode and position tracking logic, but the pin state source is seesaw-backed.
