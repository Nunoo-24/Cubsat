# System diagrams

## AE CubeSat system architecture

- [`AE_CubeSat_System_Architecture.drawio`](AE_CubeSat_System_Architecture.drawio)
  — editable diagrams.net/draw.io source. Open this file in the local desktop
  app to inspect or revise the diagram.
- [`AE_CubeSat_System_Architecture.svg`](AE_CubeSat_System_Architecture.svg)
  — scalable preview of the same system layout.

The diagram uses three domain boundaries: Environment/Test Plant, onboard
CubeSat, and Ground Station. Arrows that cross a boundary are labeled with the
data or commands exchanged. It distinguishes the synthetic host/Wokwi test
path from the physical actuator path, which is currently yaw-only. Dashed
modules and arrows identify simulation-only behavior or a hardware/interface
choice that still needs confirmation.

Treat this as a functional architecture, not a wiring diagram. Sensor pinout,
breakout electrical levels, camera variant, final physical telemetry protocol,
and motor interface still need verification against the actual kit.
