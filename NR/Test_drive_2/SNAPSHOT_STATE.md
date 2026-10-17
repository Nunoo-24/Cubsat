# Snapshot state — Test Drive 2

Refreshed from the active AE workspace on 17 October 2026. The source-workspace run passed 218/218 Python tests in 110.024 s. The refreshed
packaged copy was also verified in place: 218/218 tests passed in 116.493 s,
syntax compilation passed, and all six simulation/camera/axis reports were
regenerated. Portable C tests and the ESP-IDF build were
skipped because `cc` and `idf.py` were unavailable on PATH. Live sockets,
browser rendering, Wokwi, and physical hardware were not exercised.

The physical demonstrator scope remains yaw reaction-wheel actuation. Host SIL
supports virtual pitch/roll control; ESP32 virtual-axis commands are restricted
to simulator-framed observations. The optional asymmetric camera marker enables
simulated visual roll estimation; point-only target operation uses IMU leveling.
Model values remain provisional. Start with `README.md` and
`Context/HANDOFF.md`.
