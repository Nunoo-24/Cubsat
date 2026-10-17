# System diagrams

## AE CubeSat system architecture

- [`AE_CubeSat_System_Architecture.drawio`](AE_CubeSat_System_Architecture.drawio)
  — editable diagrams.net/draw.io source. Open this file in the local desktop
  app to inspect or revise the diagram.
- [`AE_CubeSat_System_Architecture.svg`](AE_CubeSat_System_Architecture.svg)
  — scalable preview of the same system layout.

The diagram keeps three top-level boundaries: Environment/Test Plant, onboard
CubeSat, and Ground Station. Inside the CubeSat boundary, camera, sensing,
onboard processing, actuation, and telemetry/telecommand are separate nested
domains. Processing is broken into attitude estimation, pointing/guidance, and
control/safety cards; arrows connect the specific processing cards and carry
larger labels. Increased the gaps between camera/processing cards, sensor
stages, actuation/telemetry domains, and successive CubeSat subsystem rows.
Environment and Ground Station boundaries were tightened around their content. All 29 connector labels are independent text boxes in the editable diagram,
so they can be moved, rotated, resized, and edited separately from their
arrows. The reaction-torque/body-motion text box is rotated vertically.
Simulation-only and pending hardware/interface details have distinct styles.

Treat this as a functional architecture, not a wiring diagram. Sensor pinout,
breakout electrical levels, camera variant, final physical telemetry protocol,
and motor interface still need verification against the actual kit.

## Label review copy

[`AE_CubeSat_System_Architecture_version_codex.drawio`](AE_CubeSat_System_Architecture_version_codex.drawio)
contains a wording and formatting cleanup of the current diagram labels for
comparison. Its SVG preview is included alongside it. The original diagram
remains the reference for the current user-arranged connector layout.
