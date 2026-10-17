# IMU and attitude sensing

**Role:** supply body angular rate, gravity direction, and magnetic field for telemetry and onboard yaw estimation. The estimate supports the explicit relative yaw-step mode; regular visual pointing remains camera-error based.

**Hardware described:** the user lists a “5.5V MPU-6050” breakout with a
3-axis gyro/accelerometer and a separate three-axis magnetometer. The bare
MPU-6050 chip's specified VDD is 2.375–3.46 V, with VLOGIC at 1.8 V ±5% or
VDD ([TDK/InvenSense datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)). Treat 5.5 V as a possible breakout input rating only, pending the exact schematic and revision. Do not assume the breakout's I²C pull-ups are safe for XIAO GPIO until checked.

The magnetometer's stated ±4/8/12/16 gauss and I²C/SPI description matches
the LIS3MDL family. One common Adafruit breakout variant has a regulator and
level shifting, 3–5 V VIN/logic, and I²C addresses 0x1C/0x1E ([official guide](https://learn.adafruit.com/lis3mdl-triple-axis-magnetometer)). This is a likely match, not confirmation of the board in the kit. Confirm markings and revision before wiring. The magnetometer is not required for the relative camera-pointing and rate-damping loop.

**Implementation:** [`CubeSat_ESP32/attitude.py`](../../Python_sim/CubeSat_ESP32/attitude.py) and the portable C implementation provide a gyro-integrated yaw estimate with gravity-based tilt compensation of a magnetic heading. Synthetic XYZ gyro, accelerometer, and fixed-world magnetic
field data are produced by the environment. Optional ESP-IDF MPU-6050 and LIS3MDL drivers
share the XIAO D4/GPIO5 SDA and D5/GPIO6 SCL bus via
`../../Python_sim/CubeSat_ESP32/firmware/main/sensor_bus.c`. The MPU driver
averages stationary startup gyro bias; the LIS3MDL driver reports a raw field
vector in microtesla at a Kconfig-selectable ±4/8/12/16 gauss range (±4 gauss
default). The firmware currently selects the breakout's I²C interface at
0x1C/0x1E; SPI remains a possible board interface, not an implemented driver.
Both drivers stamp their reads.

**Software check:** the simulation evaluator initializes the yaw estimate at
five combined roll/pitch/yaw poses and compares it with known plant yaw. The
current ideal synthetic vectors reproduce yaw within 0.000001° in all five
cases. A separate camera projection grid verifies the synthetic gravity/target
frames use consistent body rotations. These checks validate coordinate/sign
consistency only: the estimator shares the simulator's placeholder magnetic
reference, so neither check establishes physical sensor accuracy.

An 81-case observation-level stress screen varies gyro bias (±0.03 rad/s),
gyro noise (0, 0.05°/s RMS, and 0.01 rad/s RMS), and matched sample delay
(up to 80 ms), alongside camera bearing-error noise. The 0.05°/s point is the
TDK datasheet gyro-noise specification at a different DLPF setting; it is a
comparison input, not a prediction for this breakout. Other bias/noise values
are assumed stresses. One combined case narrowly exceeds the 2° hold limit;
see [model results](../../Verification/Simulation_Results.md).

**Provisional software axis convention:** the model uses body X=pitch, Y=yaw,
Z=roll, and the current yaw controller consumes gyro Y. Firmware currently
passes MPU-6050 Y as yaw rate. The physical sensor-to-body mounting transform
is unknown and must be measured before hardware control; remap axes/signs in
the sensor adapter to match the installed orientation.

**Limit:** this is a yaw-only complementary estimator, not a full three-axis
AHRS. Python SIL corrects yaw against the simulated fixed field; firmware
physical sensor mode suppresses magnetic correction and reports relative
gyro-only yaw until hard/soft-iron calibration, local field reference, and
sensor mounting are confirmed. Neither C code nor the physical drivers have
been compiled/bench-tested. Accelerometer gravity correction assumes limited
linear acceleration. A stale or invalid IMU sample is rejected without gyro
integration; an active yaw step returns to SAFE if attitude becomes invalid.
The estimate feeds the explicit yaw-step setpoint loop
and is also reported in telemetry; it does not replace camera error in regular
visual acquisition/tracking. The MPU-6050 source configures DLPF_CFG=3 and
SMPLRT_DIV=4 (200 Hz output); it estimates effective sample time by subtracting
4.9 ms nominal filter delay and up to 5 ms output phase. This timestamp logic
is source-reviewed but not compiled or verified on a device.

**Verify next:** build and run the C host test, then build ESP-IDF firmware.
Identify exact breakouts, check electrical levels and axes, compare
bias/range/noise against primary datasheets, and validate real samples. Calibrate
hard/soft-iron effects and assess motor interference before enabling magnetic
correction on hardware. Compare yaw error/drift against rig truth before using
the estimator in a control path.


**Tilt-formula parity:** host SIL and C firmware estimate pitch from
`atan2(-az, hypot(ax, ay))` and roll from `atan2(ax, ay)`, and accept gravity
magnitude only within 0.8–1.2 g. A Python source contract checks both firmware
formulas and thresholds against SIL. The synthetic five-pose regression checks
the SIL estimates against plant truth; C and physical-sensor execution remain
unverified. These mappings rely on the documented axis convention and still
require measurement on the assembled hardware.
