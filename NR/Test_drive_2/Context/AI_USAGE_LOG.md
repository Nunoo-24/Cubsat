# AI usage log — AE project

Keep this log with any AI-assisted project/report work and attach or reference it
as required by the course. The course syllabus permits AI for project work under
three conditions: log its use, verify generated values/formulas against cited
primary sources, and take full responsibility for submitted work. AI must not be
used in the E4 written exam.

## 2026-10-17 — Resume context and refresh full offline verification

Re-read the workspace and AE project instructions, current handoff, project
brief, software review, user prompts and notes, design description,
requirements traceability, current status guide, and current generated reports.
Confirmed that `Context/original_chat.json.txt` is empty. The supplied PDFs
remain in `Source/`, but no PDF text extraction tool is installed. The active
software worktree was treated as authoritative; archived `NR/Test_drive_*`
copies were not reread line-by-line.

Ran `outputs/Python_sim/run_verification.sh` to completion. All 218 Python
tests passed in 110.024 seconds; Python syntax compilation and all six report
generators completed. Portable C host tests and ESP-IDF build were skipped
because no C compiler and no `idf.py` are on PATH. Live sockets/browser,
Wokwi, and hardware were not exercised. The run regenerated the simulation,
camera transport, camera geometry, camera image-noise, camera detector timing,
and axis-control reports. A new report,
`outputs/reports/AE_Project_Current_State_Report_2026-10-17.md`, records the
current state, test evidence, unresolved gates, and context-reading limits.

| Date | Work assisted | AI contribution | Verification / author action |
|---|---|---|---|
| 2026-10-16 | Improve live simulator UI comparison/navigation after first user trial | Reviewed live telemetry growth and recent control/sensor states; added display-only orbit/zoom/reset to the environment canvas and a labeled simulator-truth comparison panel to the Ground Station. Updated user-facing run notes and continuity records. | Static source review only. No browser interaction or tests were run; user should hard-refresh both pages and check orbit/zoom and truth-versus-telemetry display. |
| 2026-10-16 | Separate MPU-6050 chip limits from the user-listed breakout rating | Checked the TDK/InvenSense datasheet after the user input described a “5.5V MPU-6050”. Updated hardware notes to distinguish the bare sensor supply/logic limits from an unidentified breakout’s possible VIN rating and I²C level shifting. | The chip limit is supported by the manufacturer datasheet; no breakout schematic or identity was available. Exact board supply, pull-up rail and safe XIAO wiring remain to be confirmed before connection. |
| 2026-10-16 | Verify the magnetometer described in the user prompt | Compared the prompt’s ±4/8/12/16 gauss and I²C/SPI specification with ST and Adafruit primary-source documentation. Recorded that it identifies the LIS3MDL family and is a likely match for an Adafruit breakout, while keeping vendor/revision and installed-board voltage tolerance unconfirmed. | The official Adafruit guide documents VIN/logic-level handling and I²C addresses for its own breakout; ST chip limits remain separate. Physical markings must be checked before wiring. No firmware or hardware test is implied. |
| 2026-10-16 | Align sensor/control subsystem notes with yaw-step implementation | Audited subsystem, architecture, ICD and review notes against current SIL/firmware source; corrected statements that described the one-axis estimate as telemetry-only, clarifying its use in explicit relative yaw-step feedback and preserving camera-error control for regular visual pointing. Updated PERF-07 traceability wording. | Documentation-only correction; reran all 97 Python tests, `compileall`, evaluator report generation, and a scan of 58 relative Markdown links. All passed. No new firmware verification is implied. |
| 2026-10-16 | Implement and evaluate the 45° yaw-step milestone | Added bounded relative `SET_YAW_STEP` commands to the Ground Station, environment bridge, Python SIL, and source-level ESP-IDF app. The onboard estimate drives the bounded yaw controller; completion requires ≤1° error and ≤0.5°/s for one second, with a ten-second timeout. Added telemetry, operator controls, integration tests, evaluator case, requirements traceability, and interface/design notes. | All 97 Python tests pass; `compileall` and the regenerated model report pass. The Python model enters the settle band at 5.820 s and completes at 6.820 s. ESP-IDF C is source-reviewed only; no firmware compile, emulator execution, live network, or hardware test is claimed. |
| 2026-10-16 | Verify yaw-step command integration across the Python system | Added a socket-free regression that submits UI command fields to the Ground Station HTTP handler, queues the resulting command through the simulator HTTP handler, executes it in onboard SIL, posts telemetry/acknowledgement back through the handlers, and confirms the plant reaches the step within the ten-second command timeout. Synchronized the test monotonic clocks to avoid triggering the environment watchdog artificially. | The targeted integration test passes, including command acknowledgement, terminal STANDBY mode and settle/error bounds. This validates Python component integration only; live TCP, ESP-IDF firmware, and hardware remain unverified. |
| 2026-10-16 | Motor-speed milestone feasibility and moteus velocity research | Added a deterministic best-case spin-up calculation to the model evaluation report and a regression; added a separate bounded velocity-frame encoder in the moteus C adapter and Python protocol reference, without exposing it to the operator interface. | All 73 Python tests, `compileall`, report generation, and the 54-link scan pass. Official mjbots control-mode documentation states velocity mode uses NaN position and a velocity in rev/s. The C adapter remains uncompiled and the CubeSat application does not call its velocity API. The 431 rpm bound and 62.8 mN·m calculation use provisional model values only; actual motor/wheel performance is unverified. |
| 2026-10-16 | Requirements traceability and current tool audit | Added `outputs/Verification/Requirements_Traceability.md` to map functional and performance requirements to implementation and evidence, explicitly separating model results from target/runtime/hardware verification. | Requirements were cross-checked against the supplied project brief and current implementation/report files. Re-ran all 69 Python tests, `compileall`, the scenario evaluator, and a scan of 54 relative Markdown links; all passed. Firmware, live networking, browser, Wokwi, and hardware tests remain unverified. Student/team must verify guide acceptance values and the actual kit. |
| 2026-10-16 | Project workspace setup and source review | Organized project instructions, read the supplied syllabus/project guide and input notes, and summarized the CubeSat project architecture and requirements in `Context/PROJECT_BRIEF.md`. | Requirements and numerical targets were transcribed from `Source/1-project_guide.pdf` and `Source/0-Guiao.pdf`; they remain proposed course-guide targets and must be confirmed by the student/team/lecturer. |
| 2026-10-16 | Existing software review and input-note update | Inspected the ground-station UI, UDP simulator, ESP-IDF heartbeat example, and relocation configuration; updated `inputs/Dicas.txt` and `inputs/sumury.txt` to reflect onboard ESP32 processing. | Python source syntax was checked. Runtime UDP verification was blocked by this environment's socket restriction; GUI import/runtime and firmware build/flash were not verified. Student/team must review architecture and test on the target environment/hardware. |
| 2026-10-16 | Initial design description and Level 0 ICD | Drafted `outputs/Design_Description.md` and revised `outputs/Python_sim/docs/TMTC-ICD.md` to separate the implemented loopback prototype from proposed onboard functions. | The pixel-to-bearing relation in the design draft was checked against the pinhole projection/intrinsics described in the [OpenCV 3D projection documentation](https://docs.opencv.org/5.0/main_modules/d_projection.html). It assumes calibrated, distortion-corrected imagery; actual intrinsics, mounting alignment, sign convention, and performance still require measurement. Other performance numbers are from supplied course/project materials, not independent test results. |
| 2026-10-16 | Initial software build-out | Read `inputs/Prompts.txt`, organized the existing repository into `Sim_Environment/`, `Ground_Station/`, and `CubeSat_ESP32/`, added the browser simulation scene, yaw plant, synthetic camera/IMU bridge, Python SIL and Ground Station, portable Python/C control-law references, camera-centroid primitive, subsystem/interface documents, and Wokwi starter configuration. Updated subsystem notes and corrected the stale code review. | Primary-source hardware/emulation research and the ESP-IDF/cJSON dependency decision are summarized in `references/Hardware_and_Emulation_Notes.md`. Thirty-one Python unit tests pass, including simulated regressions for proposed acquisition, hold, nudge recovery, slow-target tracking, camera-occlusion, XYZ telemetry, and stale-telemetry criteria. Python syntax compilation, Wokwi diagram JSON parsing, and managed-component YAML parsing pass. The direct in-process SIL settling check passed. Socket creation is denied here, so live HTTP UI/proxy integration could not be run. No host C compiler, ESP-IDF, or Wokwi CLI is installed; C build, firmware emulation, and HIL remain unverified. Physical constants and sensor assumptions are provisional and require team review and hardware validation. |
| 2026-10-16 | Optional onboard camera path | Researched Espressif's ESP32-S3 camera driver and the published XIAO Sense camera GPIO map. Added a Kconfig-controlled grayscale QVGA capture adapter that passes frames into the C dark-target detector; added the managed camera dependency and updated the architecture and build notes. | Seeed's official GPIO allocation matches the Espressif XIAO pin map. Espressif documents ESP32-S3/OV5640/OV3660 support and PSRAM use. Camera profile remains disabled in Wokwi; this physical camera adapter has not been compiled or hardware-tested because ESP-IDF and the board are unavailable. It currently coexists with synthetic IMU/plant data and is only a camera bring-up path, not full HIL. |
| 2026-10-16 | Optional onboard MPU-6050 path | Researched the XIAO D4/D5 pinout, ESP-IDF 5.2 I²C master API, and InvenSense register map. Added a conditional I²C driver, raw XYZ conversion, timestamps, stationary startup gyro-bias averaging, and XYZ Ground Station telemetry. | Official references support the chosen XIAO pins, register access, and sensor scaling. The driver remains disabled in Wokwi and is not compiled or bench-tested. Breakout supply/logic/pullups, address, axis alignment, calibration, and sample timing must be verified on the actual board. |
| 2026-10-16 | Low-bandwidth physical camera preview | Added the exact-size 64x48 grayscale bridge endpoint and freshness timeout, Ground Station proxy/panel, optional firmware downsampling and 1 Hz HTTP upload, and regression coverage. | All 33 Python unit tests and Python syntax checks pass. Firmware upload, HTTP runtime, and browser rendering remain uncompiled or unverified because ESP-IDF is absent and local sockets are denied. This is local simulation preview transport, not the physical radio image protocol. |
| 2026-10-16 | Optional LIS3MDL sensing path | Verified the ST register map, sensitivity, I²C addresses and data-ready behavior; added a shared MPU/LIS3MDL I²C bus, optional 80 Hz LIS3MDL driver, synthetic body-frame magnetic field, SIL/firmware telemetry, Ground Station readout, and regression coverage. | 42 Python unit tests pass; Python syntax and diagram JSON checks pass. The C/Kconfig/ESP-IDF build and physical magnetometer path remain uncompiled and unbench-tested. The synthetic field is an interface placeholder and is not fused or used by control. |
| 2026-10-16 | Three-axis simulation and target overlay documentation | Reviewed the Ground Station thumbnail overlay, clarified simulated wheel telemetry and provisional MPU axis mapping, and aligned the API/system notes with the three-axis model. | All 39 Python unit tests, Python syntax compilation, and Wokwi diagram JSON parsing pass. Browser rendering, live HTTP exchange, ESP-IDF compilation, and physical axis mapping remain unverified. |
| 2026-10-16 | Actuator interface compatibility review and subsystem map | Checked official ESP-IDF S3 TWAI/SPI guides, moteus CAN-FD and UART integration docs, and Microchip MCP2518FD data. Added the CAN-FD incompatibility finding, two candidate hardware paths, and per-subsystem output guides. | ESP32-S3 TWAI is Classical CAN and cannot exchange the moteus FD frame format; a simple transceiver is not enough. moteus documents TTL UART on r4 AUX2 at 921600 8N1 with sufficiently recent firmware. UART is an engineering inference as simplest candidate for a single controller; exact kit configuration and electrical levels remain unverified. The external CAN-FD alternative requires an FD controller and transceiver. No actuator hardware path has been implemented. |
| 2026-10-16 | Reaction-wheel dynamics in the simulation plant | Added a one-axis wheel model with opposite reaction torque, first-order motor lag, torque and speed saturation, and separate simulated actuator fields for the Ground Station. Updated scene readouts and subsystem/architecture notes. | 36 Python tests pass, including angular-momentum conservation without damping, sign, torque/speed limits, and model performance regressions. Wheel inertia 1e-4 kg·m², torque limit 0.01 N·m, speed 500 rad/s and lag 50 ms are tunable placeholders only, not measured actuator behavior. |
| 2026-10-16 | Three-axis gimbal disturbance and camera-frame model | Added decoupled roll/pitch pendulum dynamics, XYZ torque mapping (X=pitch, Y=yaw, Z=roll), full Euler camera-vector projection, projected XYZ gyro/gravity observations, three-axis cube rendering, and explicit yaw-rate field/Y-axis use in the SIL/firmware interface. | 39 Python tests pass, including XYZ torque signs, pendulum restoring behavior, camera centering at the target bearing, and prior yaw performance regressions. Euler coupling and physical mounting alignment remain unverified; all mechanical parameters are placeholders. Firmware C and browser rendering are not executable in this environment. |
| 2026-10-16 | Opt-in moteus r4 UART adapter and telemetry | Added a default-off ESP-IDF UART motor adapter with CRC-checked fdcanusb lines, 64-byte torque/query register frames, finite torque/watchdog values, arm input, status/fault parsing and Ground Station/CSV status fields. Added a host-side wire-format reference and regression tests. Official Seeed, mjbots and Espressif documentation informed pin selection and protocol details; official mjbots client source corrected the space-before-star CRC, `BF` flags and `0x50` FD padding. | 49 Python unit tests, Python syntax compilation, Wokwi JSON, 36-document local-link scan and 26-ID Ground Station DOM reference checks pass. A concurrent environment/SIL/Ground Station launch was attempted; the two HTTP servers failed because socket creation is denied in this environment, and the SIL process timed out while retrying. ESP-IDF build, Kconfig parsing, runtime UART/servo interoperability, logic levels and physical actuation remain unverified. Keep the adapter disabled until hardware and independent emergency-stop checks are complete. |
| 2026-10-16 | Workflow persistence and actuator documentation consistency | Added the requested autonomous research/build/integration/evaluation loop to the project README and corrected the remaining stale D6/D7 UART pin reference to match the adapter's configured D0/D1 pins. | Re-ran the 49-test Python suite, Python syntax compilation, Wokwi JSON parsing and 36-document local-link scan; all pass. No firmware C build or live network runtime was available. |
| 2026-10-16 | ESP-IDF build-environment audit | Inspected the retained Windows ESP-IDF cache and toolchain metadata. Confirmed ESP-IDF 6.1 and the ESP32-S3 compiler are mounted in WSL, but launching the Windows compiler fails at the denied WSL interop socket. The old ELF/cache is dated Oct 6, references the former Windows checkout, and has no compilation entry for the Oct 16 moteus adapter; it cannot validate current firmware. | Re-ran 49 Python tests, Python syntax compilation, Wokwi JSON parsing and all local links in 36 Markdown files; they pass. Current ESP-IDF build and live HTTP runtime remain unverified. |
| 2026-10-16 | SIL sensor freshness correction | Separated camera and IMU telemetry ages while preserving a strict max-age gate for control; missing, negative, non-finite, or unparseable ages now inhibit torque and invalid numeric ages serialize as null. Documented the rule in the HTTP API and added regression coverage. | 52 Python tests pass, including three new timestamp cases; Python syntax compilation and Wokwi JSON parsing pass. Live process HTTP remains unavailable in this sandbox. |
| 2026-10-16 | ESP-IDF sensor freshness alignment | Applied independent finite/nonnegative camera and IMU age checks to the firmware controller input, report invalid ages as null, and timestamp physical grayscale frame capture locally so it no longer consumes synthetic camera age. Updated camera/architecture docs. | Re-ran 52 Python tests, syntax compilation and Wokwi JSON parsing; all pass. The ESP-IDF C changes remain source-reviewed because WSL cannot launch the mounted Windows toolchain. |
| 2026-10-16 | ESP-IDF peripheral component dependencies | Inspected the installed ESP-IDF 6.1 component CMake and header layout. Replaced the legacy umbrella dependency with explicit conditional UART, GPIO and I2C driver components, raised the supported minimum to 5.3, and added official Espressif versioned API references. | Re-ran 52 Python tests, Python syntax compilation, Wokwi JSON and component YAML checks; all pass. Confirmed the declared component directories exist in IDF 6.1. Firmware CMake configure/build is still unverified because WSL cannot invoke the Windows ESP-IDF tools. |
| 2026-10-16 | Socket-free HTTP boundary integration checks | Added an in-memory HTTP request harness and exercised environment observation, command and actuator routes; Ground Station telemetry proxy/CSV logging and command forwarding; and a complete environment → SIL → environment actuator exchange. Hardened Ground Station command handling so JSON arrays/null return HTTP 400. | 61 Python tests pass, plus syntax, diagram JSON, managed-component YAML and local Markdown-link checks. These checks do not prove live TCP or concurrent multi-process runtime, which remain unavailable in this sandbox. |
| 2026-10-16 | Wokwi configuration source review | Compared the current `wokwi.toml` and XIAO ESP32-S3 `diagram.json` with Wokwi's ESP-IDF integration and diagram-format docs. The flasher manifest/ELF paths, supported board part, and octal PSRAM annotation match documented configuration. | JSON and links parse, but Wokwi itself has not been run because the CLI/extension is unavailable in this environment. |
| 2026-10-16 | ESP-IDF Kconfig profile validation | Ran the installed ESP-IDF 6.1 Kconfig generator against the framework root and current project options for the default and camera+MPU-6050+moteus profiles. Removed the UTF-8 BOM from `sdkconfig.defaults` after the parser showed that IDF ignored the target setting. | Both profiles parse and the physical profile resolves the expected enabled options. This validates Kconfig only; CMake and firmware compilation remain unverified. |
| 2026-10-16 | ESP-IDF execution-path audit | Confirmed the ESP-IDF 6.1 VS Code extension and compiler are mounted on Windows, but Linux cannot use the Windows Python environment because `pydantic_core` is a Windows binary extension, while WSL interop cannot launch native Windows executables in this sandbox. | The pure-Python IDF Kconfig generator runs and validates default/physical profiles. `idf.py` build and Wokwi execution remain unavailable here. |
| 2026-10-16 | Actuator status documentation consistency | Audited the supplied prompt, project brief, architecture and subsystem notes against the current adapter source. Corrected stale claims that no physical motor command path existed; the adapter is optional/default-off and remains uncompiled/unverified. | Targeted contradiction search found no remaining stale statements; all 61 Python tests and 36 Markdown local-link checks pass. |
| 2026-10-16 | Controller configuration fail-closed validation | Reviewed the Python and C yaw-controller input/configuration guards. Added rejection of non-finite gains/limits, negative gains, and nonpositive torque/age/rate bounds; invalid configuration now produces zero torque and `INVALID_CONFIG`. Added Python regression coverage and a C host-test assertion. | All 62 Python tests and Python syntax compilation pass. Host C compilation was checked but `gcc`, `clang`, and `cc` are absent. C and ESP-IDF behavior therefore remain source-reviewed, not compiled. |
| 2026-10-16 | Supervisory command replay handling | Audited bridge command acceptance against the onboard monotonic command filter. Added increasing-ID enforcement, identical retry acceptance while queued, rejection of conflicting/stale IDs, and documented ten-second expiry and process-local ID state. | Added regression assertions; all 62 Python tests, Python syntax compilation, and local Markdown links pass. The contract is for the local simulation bridge; physical-link replay/persistence remains undefined. |
| 2026-10-16 | JSON command ID precision guard | Checked how the Python bridge IDs cross into cJSON's double-valued JSON numbers. Restricted IDs across bridge, host SIL, and firmware to integral values in 1..2^53−1, validating in firmware before integer conversion. | Added tests for boolean, fractional and out-of-range IDs; all 63 Python tests and Python syntax compilation pass. C remains uncompiled. |
| 2026-10-16 | Firmware control fault serialization | Checked enum additions against firmware telemetry serialization after adding `INVALID_CONFIG`. Added the missing mapping so firmware reports the same fault name instead of `UNKNOWN_FAULT`. | Source inspection confirms enum-to-string coverage; C compilation is unavailable in this WSL image. The Python suite is rerun for the project state but does not execute this firmware serializer. |
| 2026-10-16 | Expired supervisory command retry | Compared bridge retry handling with its ten-second delivery expiry. Retries of queued but expired commands could receive a false acceptance; added an original-timestamp expiry check and regression coverage. | All 63 Python tests and syntax compilation pass. Updated HTTP API, ICD and code review; physical link behavior remains unspecified. |
| 2026-10-16 | Optional actuator command-path review | Traced the Kconfig torque limit through the UART adapter clamp and frame fields, and checked SAFE/STANDBY/fault zeroing plus timeout behavior. | Source inspection confirms the software path is bounded to configured 1–20 mN·m and carries a finite 100 ms drive watchdog. No C build, UART timing test, electrical check, or physical stop test was possible. |
| 2026-10-16 | 50 Hz control-loop scheduling | Compared the ESP-IDF loop's relative delay with the host SIL deadline behavior and inspected ESP-IDF 6.1 FreeRTOS Kconfig/API sources. Switched firmware to `xTaskDelayUntil()`, set the 100 Hz tick default, added the tick divisibility guard, and removed SIL catch-up bursts after overruns. | Added SIL deadline-policy coverage; all 64 Python tests, Python syntax compilation, and 36 Markdown local-link checks pass. Firmware scheduling is source-reviewed but uncompiled; target cycle time and jitter need measurement. |
| 2026-10-16 | FreeRTOS scheduling Kconfig recheck | Ran the installed ESP-IDF 6.1 Kconfig generator using current project defaults for the default and physical camera+MPU-6050+moteus profiles. | Both profiles resolve successfully with a 100 Hz FreeRTOS tick and expected enable flags. This does not run CMake or compile firmware. |
| 2026-10-16 | Command acknowledgement validation | Reviewed bridge acknowledgement endpoints and actuator telemetry update order. Restricted acknowledgements to IDs issued by the bridge, valid statuses, and required rejection reasons; made embedded acknowledgement validation atomic with telemetry acceptance. | Regression verifies an invalid embedded acknowledgement returns 400 without mutating telemetry and an unissued ID is rejected. All 66 Python tests and syntax compilation pass; docs now specify the contract. |
| 2026-10-16 | Measured model evaluation report | Added a repeatable runner and human-readable results report for guide-derived acquisition, hold, nudge, slow-target and camera-occlusion scenarios. The report explicitly labels the limits of model-only evidence. | All five model scenarios pass with recorded numerical outcomes. The runner exits nonzero on a failed criterion. All 66 Python tests, syntax compilation, and 38-document local-link checks pass. |
| 2026-10-16 | Full guide milestone coverage in evaluation | Cross-checked the results report against the earlier motor-speed, attitude-estimation/drift, and 45° step milestones in the project brief. Listed them as not evaluated with the missing capability stated; added a boundary regression for the synthetic camera's 30° field limit and clarified the 30° acquisition result is an edge case. | All 67 Python tests pass, including 29.9° visible / 30.1° outside FOV. The five model-level pointing scenarios still pass; syntax and Markdown link checks pass. |
| 2026-10-16 | Atomic environment control updates | Found that valid fields could be committed before a later invalid scene-control field returned HTTP 400. Refactored `Environment.apply()` to validate the entire payload before committing any field and added an HTTP-handler regression. | All 68 Python tests, syntax compilation, and the model evaluation report pass. Updated the Simulator HTTP API contract. |
| 2026-10-16 | Strict scene-control request schema | Prevented unknown fields and standalone torque duration from silently receiving successful no-op responses. Added validation and regression cases. | All 69 Python tests, syntax compilation, and model report pass; HTTP API documentation lists the field and dependency rules. |

## Author responsibility

The student/team must check the technical content, verify component details and
requirements against primary sources and supplied hardware, correct mistakes,
and take responsibility for the final submitted work. This log records AI
assistance; it does not replace that review.
| 2026-10-16 | Integrate bounded wheel-speed commissioning path | Connected `SET_WHEEL_SPEED` across Ground Station, the simulation bridge, Python onboard reference, and source-level ESP-IDF command/actuator path. Added SAFE/STANDBY gating, 0–50 rev/s and 0.5–5 s bounds, zero-speed stop and timeout status, telemetry/CSV/UI fields, and an in-memory integration regression. Updated the interface and traceability documents. | All 83 Python unit tests pass; Python `compileall` passes; model evaluation regenerated with five pointing scenarios passing. Wheel speed performance remains unverified: the placeholder model reaches 431 rpm in 0.5 s; ESP-IDF firmware is uncompiled, and no real motor/wheel data or bench test is available. |
| 2026-10-16 | Retain commissioning completion status | The Ground Station polls telemetry slower than the 50 Hz onboard cycle, so a one-cycle `STOPPED` or `STOP_TIMEOUT` value could vanish before display. Python SIL and ESP-IDF source now retain the last terminal status until another wheel-speed request begins. Added a Python regression and updated the actuator/API/hardware/code-review/handoff docs. | All 84 Python tests pass, `compileall` passes, and the model report regenerates. The C status-latch change is source-reviewed only; ESP-IDF build and live Ground Station polling remain unavailable. |
| 2026-10-16 | Correct speed-test telemetry persistence and resync project status | A speed-test completion value could disappear between 50 Hz onboard telemetry cycles and slower Ground Station polling. Added a latched terminal status to Python SIL and ESP-IDF source, covering both stopped and stop-timeout cases; expanded the regression. Corrected the CubeSat header comment, actuation/hardware subsystem notes, design description, API docs, code review and project brief where they still described velocity commissioning as future work. | All 84 Python tests pass; Python `compileall`, report generation and 54-link audit pass. A direct environment server launch was attempted and failed at socket creation (`PermissionError: Operation not permitted`). ESP-IDF and Wokwi remain unavailable; the C latch is source-reviewed only. |
| 2026-10-16 | Add and evaluate onboard yaw attitude estimate | Implemented a project-frame one-axis complementary yaw estimator in Python SIL and portable C. It integrates gyro Y and corrects drift with tilt-compensated magnetic heading when a valid calibrated reference is available; SIL uses the known synthetic field, while physical firmware remains gyro-only until calibration/mounting are confirmed. Added telemetry/CSV/dashboard fields, C host test source, Python unit tests, a 60 s truth-referenced model scenario, and updated design/ICD/sensor/traceability/handoff docs. | All 90 Python tests pass, `compileall` and model-report generation pass. Synthetic field result: 0.068° peak yaw error and 0.050° final drift over 60 s. This is yaw-only model evidence and does not validate C compilation, three-axis attitude, physical magnetic calibration, firmware, or hardware. |
| 2026-10-16 | Use physical camera capture time for freshness | Checked Espressif's official `camera_fb_t` definition: its `timestamp` is the time since boot of the first DMA buffer in the frame. Updated the XIAO camera adapter to compute frame age from that timestamp and reject zero/future values, so acquisition and detector processing latency count toward freshness. Clarified the LIS3MDL Kconfig help and firmware notes. | Source reviewed against the official esp32-camera header. Python checks do not compile this C path; ESP-IDF build and hardware camera timing remain unverified. |
| 2026-10-16 | Recheck ESP-IDF build environment and conditional firmware graph | Rechecked mounted ESP-IDF compiler, `idf.py`, Docker/Podman, and Linux C compiler availability; the ESP32-S3 Windows compiler exists but WSL cannot launch even `cmd.exe` (`UtilBindVsockAnyPort` socket failure). Inspected current conditional component registration, sensor/actuator gates, camera age path, shared I2C setup, and moteus UART exchanges for source-level consistency. | No C compilation became available. The source review is not a firmware build or target test; proceed to a working ESP-IDF host as the next build gate. |
| 2026-10-16 | Align dashboard yaw-rate fallback with body frame | Requirements review found that the Ground Station displayed gyro Z as yaw when `yaw_rate_rad_s` was absent, while the project convention and SIL/firmware define yaw on body Y. Corrected fallback to `gyro_y_rad_s` and added an HTTP-handler assertion against the served dashboard. | All 98 Python tests pass; `compileall` passes; model evaluation report regenerated. Dashboard HTML logic is checked at source level; live browser rendering remains unavailable because this sandbox denies sockets. |
| 2026-10-16 | Preserve target-loss gaps in Ground Station history | Changed the Ground Station plot to retain one point per unique telemetry sample on the simulation-time axis and break the line when target observations are invalid. This makes camera occlusions visible instead of connecting measurements across missing intervals. Updated Ground Station and software READMEs and added served-dashboard regression checks. | All 99 Python tests pass; `compileall` passes; model report regenerated. Browser rendering remains unverified because local sockets are denied and no browser runtime is installed. |
| 2026-10-16 | Pair camera preview with its own telemetry measurement | Found that the operator overlay used the newest polled telemetry on a thumbnail that may be a second older. The bridge now stores target validity/pixel error with the uploaded frame and returns them in response headers; the Ground Station proxy preserves those headers and draws the paired measurement. Updated camera/API/Ground Station docs and added pairing/proxy tests. | All 101 Python tests pass; `compileall` passes; model report regenerated. Browser rendering and physical camera timing remain unverified. |
| 2026-10-16 | Interim preview-offset scale attempt (superseded) | An initial calculation incorrectly treated the 320x240 display canvas as 64x48. The following camera geometry audit corrected this: the thumbnail is enlarged 5x onto a canvas matching original QVGA dimensions, so telemetry pixel offsets require no extra factor. The current code and interface docs use the corrected 1:1 mapping. |
| 2026-10-16 | Correct preview coordinates and include vertical camera error | Rechecked the actual Ground Station canvas size and corrected the prior scaling assumption: the 64x48 image is displayed on a 320x240 canvas, so QVGA-pixel offsets map 1:1 on screen (no 5x offset multiplier). Added vertical camera error end to end: environment convention is converted to image coordinates (positive Y down), the C detector's X/Y offsets are emitted in telemetry, the bridge pairs both with the thumbnail, and Ground Station draws the marker in both axes. Updated the protocol/subsystem docs. | All 103 Python tests pass, including simulated Y-axis sign, preview metadata pairing/proxy, and UI coordinate mapping; `compileall` passes; model report regenerated. ESP-IDF C compilation and browser rendering remain unverified. |
| 2026-10-16 | Correct preview display geometry and carry vertical camera error | After rechecking HTML image/overlay canvas attributes, corrected a mistaken interim assumption: QVGA error offsets map 1:1 to the 320x240 display canvas even though image bytes are 64x48, because the image is enlarged five times. Added camera Y telemetry end to end with a documented sign conversion (environment positive up to image rows positive down), physical centroid output, preview headers, and 2D Ground Station marker. Ensured C variables are conditionally declared for camera-on/off profiles and camera telemetry is emitted in either profile. | 103 Python tests pass, `compileall` passes, and model report regenerated. C changes are source-reviewed but uncompiled; browser rendering remains unverified. |
| 2026-10-16 | Verify and document Wokwi project configuration | Compared `wokwi.toml`, `diagram.json`, and the CMake project name with current official Wokwi ESP-IDF setup docs. Confirmed the declared ESP-IDF flasher manifest/app ELF pattern and `board-xiao-esp32-s3` board type. Added the supported `idf.py wokwi` route for ESP-IDF 6.0+ with `idf-wokwi` installed and cited the official docs. | Configuration is source-reviewed, not run: ESP-IDF build, Wokwi extension/CLI, and local WSL Windows interop are unavailable. |
| 2026-10-16 | Recheck native C compiler acquisition route | Searched the current PATH and filesystem for GCC, Clang, TinyCC, Zig, ESP-IDF, and a host compiler; none are installed. Confirmed the mounted Windows ESP32-S3 GCC exists but requires WSL interop, which has already failed. Tried downloading the Ubuntu GCC package without system installation; DNS resolution for `archive.ubuntu.com` failed. | No compiler package or source tree was changed. Firmware compilation remains unavailable in this environment. |

| 2026-10-16 | Firmware sensor/actuator source audit and verification refresh | Reviewed MPU-6050 range/configuration and still-at-startup bias assumption, LIS3MDL 80 Hz/±4 gauss/fresh-cache behavior, and moteus UART CRC/frame/watchdog handling. Re-ran the integrated Python suite and regenerated model results. | All 103 Python tests pass and `compileall` passes. No JavaScript runtime/browser automation, ESP-IDF compiler, socket permission, or physical kit was available; firmware and live runtime remain unverified. Hardware wiring, calibration, and actuator safety assumptions need confirmation before enabling the adapter. |

| 2026-10-16 | Remove duplicate camera observation member | Firmware source audit found `error_y_px` declared and assigned twice in the physical camera path. Removed the duplicate declaration and assignment, then checked struct member uniqueness and end-to-end references. | Targeted source check passed; all 103 Python tests pass, `compileall` passes, and model results were regenerated. ESP-IDF build remains unavailable, so no C compilation claim is made. |

| 2026-10-16 | ESP-IDF profile dependency recheck | Inspected firmware CMake conditional source/dependency registration and Kconfig gates for the default, camera, IMU, magnetometer and moteus profiles. Rechecked available build tools and mounted ESP-IDF paths. | Source/configuration dependencies are internally aligned; no `idf.py`, host GCC/Clang, Wokwi CLI, or ESP-IDF toolchain path was available. No build claim is made; profile compilation remains outstanding. |

| 2026-10-16 | Align system design description with implementation | Audited the human-facing architecture against SIL/firmware/Ground Station sources and corrected stale estimator, detector, mode, protection, telemetry and preview descriptions. Explicitly documented remaining limits including no detector confidence/multi-object rejection and no measured wheel-speed limit in normal pointing. | Relative Markdown link audit passes across all 39 project Markdown files. Documentation-only change; no code tests rerun. ESP-IDF build and runtime/hardware checks remain outstanding. |

| 2026-10-16 | Simulation UI/API/plant consistency and regression refresh | Compared target XYZ/scale, XYZ torque pulse, and camera-occlusion controls through the UI, `Environment.apply()`, API contract and plant axis mapping. Re-ran all documented Python modules and regenerated simulation results. | No interface mismatch found. All 103 tests pass and `compileall` passes. This does not validate live sockets/browser, firmware compilation, Wokwi, or physical hardware. |

| 2026-10-16 | Consolidated project status and run report | Prepared `outputs/reports/AE_Project_Status_and_Run_Guide.md` with current status, verified evidence, commands to run each component and tests, and outstanding firmware/emulation/live-network/hardware gates; indexed it in the AE README. | All 11 relative links in the report resolve. Report is based on the current README, handoff, traceability matrix and results file; no new software tests or runtime checks were performed for this reporting task. |

| 2026-10-16 | Resume simulation/SIL work and refresh checks | User resumed the simulated project scope. Re-read startup instructions and prompt, inspected existing simulation integration tests, and reran the documented Python suite and syntax compilation. Recorded the requested progress estimate and prompt-mapped status tree for the next report. | 103 Python tests pass; `compileall` passes. Live sockets, browser, ESP-IDF, Wokwi and physical hardware were not tested. |

| 2026-10-16 | Use simulation clock for scene events | Replaced wall-clock torque-pulse/occlusion timers with plant simulation time and proportionally integrated a torque pulse crossing a step boundary. Updated UI labels and API/system notes; adapted model scenarios to use simulation time. | All 106 Python tests pass; `compileall` and deterministic scenario evaluation pass. Live server launch still fails because socket creation is denied; browser and network behavior remain unverified. |

| 2026-10-16 | Render host-SIL camera image for Ground Station | Added a labeled 64x48 synthetic grayscale target image from fresh `cubesat_sil` telemetry, paired it with reported X/Y error, and preserved unavailable status for missing ESP32 camera frames. Updated HTTP proxy behavior for invalid-target frames and refreshed camera/Ground Station/system documentation. | All 110 documented Python tests pass and `compileall` passes. Tests cover image target-offset sign, endpoint BMP/source/error headers and Ground Station proxy behavior. Live browser rendering remains unverified because loopback socket creation is denied. |

| 2026-10-16 | Harden Sim Environment operator controls | Replaced implicit browser named-element access with explicit input lookups. Added clear UI feedback for server rejection and unavailable simulation links; documented that the SIL camera preview is a marker rendering rather than a detector input image. | All 111 documented Python tests pass and `compileall` passes. The UI handler test verifies served markup and control wiring; no JavaScript runtime or browser is installed, and socket creation remains denied, so live browser behavior is unverified. |

| 2026-10-16 | Exercise image detection in the simulated onboard path | Added observation-sequenced raw QVGA frame generation and retrieval. Host SIL processes the frame with the Python detector; default Wokwi firmware source streams the frame into the portable C detector; telemetry identifies source and sequence, and Ground Station preview downsamples the same image. Updated API/ICD/design/traceability and subsystem documentation. | All 116 Python tests pass; `compileall` and deterministic scenario evaluation pass. In-memory 500-cycle environment→detector→controller→plant timing: 4.564 ms mean, 5.098 ms p95, 9.070 ms max vs. 20 ms target, excluding sockets. Forty Markdown files have zero broken relative links; six static C camera-path checks pass. C/Wokwi build/run, live socket/browser, and hardware remain unverified. Official Wokwi gateway and ESP-IDF HTTP event references were added to research notes. |

| 2026-10-16 | Include simulated image age in onboard stale-sensor protection | Host SIL measures age from cycle request start through frame retrieval and detection; Wokwi C measures through fetch/detection. Both feed the 100 ms stale-sensor guard. Regression cases verify stale and future-dated frames fault closed with zero torque. | All 118 discovered Python tests pass; `compileall` and model evaluation pass. A fresh 500-cycle in-memory path measured 4.612 ms mean, 5.414 ms p95, 9.968 ms max vs. 20 ms target; excludes HTTP/Wi-Fi. C path remains uncompiled/unrun; live networking and browser remain unavailable. |

| 2026-10-16 | Refresh project status evidence | Re-ran the exact nine-module test command documented in `outputs/Python_sim/README.md`, Python syntax compilation, and deterministic model evaluation. Corrected stale current test/model figures and paused-state wording in the requirements matrix, status report, and handoff. | 118 tests pass; `compileall` and scenario report generation pass. No implementation code changed. Socket creation remains denied, so independent live-process communication is still not demonstrated. |

| 2026-10-16 | Record simulated camera transport budget | Reviewed host SIL and Wokwi loop schedules against the raw QVGA endpoint size. Documented the configured 30.72 Mbit/s camera payload at 50 Hz and added a live gateway throughput/latency/deadline gate to the API, architecture, camera notes, traceability, and handoff. | Calculation uses 320×240×8 bits × 50 frames/s; it is not a throughput measurement. No code or tests changed. Wokwi gateway behavior is still unverified. |

| 2026-10-16 | Align host SIL and firmware command semantics | Fixed Python SIL rejection reasons for invalid modes and made wheel-speed argument keys exact to match firmware validation. Fixed C yaw-step telemetry to emit null before a target exists or after explicit abort. Documented the shared rejection/null semantics. | Three regressions cover invalid mode, extra speed-test fields, and explicit yaw-step abort. Control module tests and all 121 Python tests pass, and `compileall` passes. C source reviewed only; ESP-IDF/host C build unavailable. |

| 2026-10-16 | Fail safe on stale attitude during yaw-step commands | Aligned host SIL with firmware's rejection of stale/invalid IMU samples and SAFE transition during an active yaw step. Aligned timeout ordering and null target/error telemetry in C, updated API and TM/TC descriptions, and added stale-IMU and timeout regression tests. | 29 control tests and all 123 Python tests pass; `compileall` and deterministic scenario-report regeneration pass. Firmware C edits remain source-reviewed and uncompiled because ESP-IDF and host C tools are unavailable. |

| 2026-10-16 | Align Python and C yaw-controller guards | Added the C controller's configurable absolute-rate limit to Python with the same 20 rad/s default and aligned fault precedence for invalid configuration versus non-finite sensors. Added Python and portable C regression cases. | 31 Python control tests and all 125 Python tests pass; `compileall` and model evaluation pass. The portable C test source was updated but not compiled because GCC/Clang are unavailable. |

| 2026-10-16 | Align non-finite acceleration handling in yaw estimators | Changed Python to reject non-finite acceleration without integrating the paired gyro sample, matching portable C. Added Python and C regression cases and documented the active yaw-step SAFE response to invalid attitude. | 37 attitude/control tests and all 126 Python tests pass; `compileall` and deterministic scenario evaluation pass. Portable C and ESP-IDF test sources remain uncompiled because no C compiler/toolchain is available. |

| 2026-10-16 | Align camera detector input validation | Made Python reject non-integer detector parameters, thresholds outside 0–255, and max dark-area bounds above image size, matching C's fixed-width API and validation. Added Python and portable C test cases and documented default detector values as provisional. | 42 attitude/control/camera tests and all 127 Python tests pass; `compileall` and model evaluation pass. C test source remains uncompiled without GCC/Clang. |

| 2026-10-16 | Guard standalone reaction-wheel model stepping | Added direct input validation to `ReactionWheel.step()` and `step_speed_target()` so finite-step and torque requirements hold when the wheel model is exercised independently of `YawPlant`. | Plant model tests and all 128 Python tests pass; `compileall` and deterministic model evaluation pass. No plant parameters or dynamics changed. |

| 2026-10-16 | Match camera reference validity to rendered frames | Replaced ideal continuous-area validity with the retained QVGA frame's actual dark-pixel count, preserving raw visible frames for undersized and oversized targets. Optimized disk rasterization by row spans and added tests for small, clipped, and oversized targets. | 56 focused environment/model tests and all 130 Python tests pass; `compileall` and scenario evaluation pass. Pointing/model outcomes unchanged. C and live camera transport remain unverified. |

| 2026-10-16 | Revert camera border rejection after acquisition regression | Tried rejecting all image-border-touching dark masks in simulation and portable C. Full model tests showed this blocks the guide's 30° FOV-edge acquisition because no target-search behavior exists. Reverted that policy, kept exact rendered-raster area validity, and updated subsystem/API/traceability notes to record the remaining clipped-centroid bias and future mitigation. | All 130 Python tests pass, `compileall` passes, and deterministic scenario evaluation passes, including 30° acquisition in 1.600 s. C remains uncompiled; live network/browser, Wokwi, and hardware remain unverified. |

| 2026-10-16 | Correct synthetic target size projection | Aligned rendered circular-marker size with the camera's pinhole position projection by using optical-axis depth instead of Euclidean slant range; safely handle targets at/behind the camera plane. Clarified that the synthetic scene uses a camera-facing marker, not an oriented poster. | 48 environment tests and all 132 Python tests pass; `compileall` and all seven deterministic model scenarios pass. The change is a synthetic geometry correction, not optical calibration or hardware evidence. |

| 2026-10-16 | Bound synthetic camera transport for SIL and Wokwi | Added lossless RLE1 encoding of the sequence-paired QVGA image, an 8,192-byte endpoint/client bound, strict Python/C decoders, and preserved the raw inspection endpoint. Host SIL and default Wokwi reconstruct the complete raster before the existing centroid detection. Updated run/API/ICD/camera/traceability docs. Checked the official ESP-IDF HTTP client event docs and made the C client validate sequence, encoding and dimensions from response headers. | Documented unit suite and full discovery: 142 tests pass; `compileall` and deterministic model report pass. Default frame is 91 encoded bytes (36.4 kbit/s payload at 50 Hz); a 60-case scene sweep remained below the 8,192-byte cap (1,447-byte maximum); local 300-sample encode mean/p95 4.289/5.118 ms and decode mean/p95 0.015/0.021 ms. This is not a network/Wokwi measurement. Portable C test added but not run; no GCC/Clang installed. |

| 2026-10-16 | Verify RLE1 bridge-to-SIL camera integration | Added a socket-free integration test that dispatches the actual environment RLE1 endpoint through the host SIL client's URL/headers/decoder, checks exact QVGA pixel reconstruction and detector output, then accepts telemetry and verifies the paired Ground Station preview. Updated the test command description, traceability and status evidence. | The integration test passes; full discovered suite has 143 passing tests, `compileall` passes and deterministic scenario evaluation passes. This verifies the Python handler/client path in memory, not local TCP, ESP32 C, or Wokwi. |

| 2026-10-16 | Make synthetic camera transport evidence reproducible | Added `Verification.evaluate_camera_transport`, a generated transport report, and checks that round-trip 60 depth/bearing/target-size scene combinations below the 8,192-byte RLE1 cap. The report measures local Python encoder/decoder timing but explicitly excludes rendering, HTTP, Wokwi, and ESP32 timing. Updated the run commands, Verification README, camera subsystem note, traceability, review, and handoff. | The report generated successfully: 60/60 frames reconstruct exactly, 10–1,447 encoded bytes, default frame 91 bytes. The documented unit suite and full discovery now contain 145 tests; timing values vary with host load and are not a real-time claim. |

| 2026-10-16 | Show stale sensor samples clearly in the Ground Station | The dashboard previously displayed a valid target/IMU flag as current even when its sensor age exceeded the onboard controller's 100 ms limit. Added explicit stale labels with sample age, hid stale direction error from the current scene, and broke the history plot until both camera and IMU samples are fresh. Updated Ground Station documentation and handoff. | All 146 discovered Python tests pass; `compileall` and deterministic scenario evaluation pass. The served-HTML regression checks the threshold and gap logic, but browser JavaScript rendering and live sockets remain unverified. |

| 2026-10-16 | Verify passive roll/pitch dynamics in the plant model | Added a no-external-torque 20 s free-decay scenario for the model's provisional linear pendular roll/pitch axes, a report row with final angles/energy retention, and a unit regression confirming X/Y/Z torques do not drive other axes in the decoupled model. Added SIM-01 traceability and updated simulation/project documentation. | The documented module suite and full discovery both pass 148 tests; `compileall` and deterministic report generation pass. Each passive axis retains 13.978% of initial mechanical energy under the current parameters. This is qualitative model behavior, not hardware calibration or a guide acceptance threshold. |

| 2026-10-16 | Check yaw estimator with combined tilt | Added simulator/SIL coverage at five known roll/pitch/yaw poses, checking accelerometer and magnetic-vector magnitude invariance, gyro axis mapping, and tilt-compensated yaw against plant truth. Added a reproducible row to the model report and updated PERF-07, the sensor note, and verification docs. | The documented module suite and full discovery both pass 150 tests; `compileall` and deterministic report generation pass. Maximum error is 0.000000° for ideal vectors that share the estimator's fixed-field assumption; this checks software coordinate consistency only, not real sensor accuracy. |

| 2026-10-16 | Verify tilted camera projection | Compared the simulator's camera X/Y pixel offsets at five combined roll/pitch/yaw poses with an independently assembled rigid-body matrix, and added a reproducible model-report result plus a projection regression. Added SIM-02 traceability and updated camera/sensor subsystem notes. | The documented module suite and full discovery both pass 152 tests; `compileall` and deterministic report generation pass. Maximum rounded-pixel discrepancy is zero in the synthetic geometry; physical focal length, distortion, target orientation and camera mounting remain unverified. |

| 2026-10-16 | Document yaw-only tilted pointing limitation | Added a six-second tilted SIL diagnostic with brief X/Z disturbance torques and 300 valid target samples. Yaw reduces horizontal camera error from -107 px to -6 px (-1.24°), while 43 px (8.82°) vertical error remains because pitch/roll are not commanded. The report labels this as a limitation diagnostic rather than a passed acceptance test. | The documented suite and full discovery pass 153 tests; `compileall` and deterministic scenario report generation pass. This demonstrates current model scope only; physical 2-axis control behavior is unverified. |

| 2026-10-16 | Sweep synthetic onboard camera detection across yaw field of view | Added a report evaluator and regression for the default 0.1 m target at 2 m depth over 13 horizontal bearings from -30° to +30°. All 13 ideal frames are detected; clipped-edge centroid error is at most 2.91 px. Updated camera subsystem, verification, traceability, and status notes, with explicit limits for physical detection rate and other marker sizes/ranges. Cross-checked all XIAO Sense camera GPIO assignments against Seeed's published pin table and recorded the source. | The documented suite and full discovery pass 154 tests; `compileall` and deterministic scenario report generation pass. This is a renderer/detector model check and board-map source review only; no physical camera, firmware build, or Wokwi timing evidence. |

| 2026-10-16 | Screen yaw timing sensitivity at model-parameter corners | Added a deterministic direct controller/plant sweep over 32 corners formed by 0.5×/2× yaw inertia, damping, wheel torque, motor lag, and paired proportional/derivative gains. All corners meet the proposed 30° acquisition (<6 s) and 20° nudge recovery (<4 s) timings; worst cases are 3.08 s and 2.48 s. Documented that perfect measurements and the simplified plant exclude end-to-end uncertainty. Updated the status report with a clearly labeled ~65% planning estimate and a component/evidence tree. | The documented suite and full discovery pass 155 tests; `compileall` and deterministic report generation pass. This is model sensitivity evidence only, not calibrated parameters, complete SIL robustness, firmware, or hardware verification. |

| 2026-10-16 | Screen onboard control against observation noise and latency | Added a deterministic 54-case Python SIL observation screen varying camera bearing-error noise, gyro bias/noise, and matched camera/IMU delay up to 80 ms. Acquisition and nudge timing pass in 54/54 cases; the 60 s hold limit passes 53/54, with a 2.025° near-miss under the strongest combined assumed stress. Documented that noise is injected after detection and inputs are not measured hardware characteristics. | The documented suite and full discovery pass 156 tests; `compileall` and deterministic report generation pass. This is not image-noise testing, calibrated sensor evidence, firmware, or hardware verification. |

| 2026-10-16 | Ground SIL sensor stress inputs and estimate MPU-6050 effective sample time | Expanded the observation-noise grid from 54 to 81 cases with a 0.05°/s RMS gyro-noise comparison point from the TDK datasheet; updated the MPU driver timestamp to account for the configured DLPF nominal delay and 200 Hz output phase, and refreshed current reports/handoff. | Datasheet values and DLPF timing are linked in the hardware notes. The screen now passes acquisition/nudge 81/81 and hold 80/81, with a 2.034° miss at 40 ms under combined assumed stress. The C timestamp change is source-reviewed only; no compiler, ESP-IDF, live runtime, or hardware verification is available in this environment. |

| 2026-10-16 | Verify complete simulated visual acquisition across component handlers | Added an in-memory integration regression that starts with a 30° scene target, fetches environment observations and sequence-matched RLE1 frames, decodes them, then runs the SIL detector/controller, returns bounded actuator telemetry to the plant, and reads the resulting telemetry through the Ground Station handler. | The targeted regression passes, reaching ≤2° by both detector-derived camera error and plant truth within the six-second model criterion, with bounded torque. Full documented suite, discovery and compileall were rerun after the change. This does not prove live TCP sockets, browser rendering, Wokwi, firmware, or hardware. |

| 2026-10-16 | Add source-level browser UI DOM contract checks and retry local plant launch | Added two repeatable Python checks for duplicate HTML IDs, JavaScript element lookups, and UI endpoint names against their matching HTTP handler source in the simulation and Ground Station interfaces; added the module to the documented suite. Retried the actual simulation service startup. | Both UI checks pass; they only check source wiring and do not render a browser. The plant process again fails at TCP socket creation with `PermissionError: Operation not permitted`. Full suite and discovery pass 159 tests, compileall passes, and no live networking/browser claim is made. |

| 2026-10-16 | Analyze the baseline SIL hold miss without retuning hardware defaults | Parameterized the deterministic 81-case noise/latency evaluator by PD gains and added a reproducible eight-point gain sensitivity sweep to the report, plus a regression for Kp=0.10 N·m/rad, Kd=0.04 N·m·s/rad. The baseline remains Kp=0.08, Kd=0.04; no production/Python/C gain was changed. | Seven of eight candidates pass acquisition, nudge and hold (best tested 1.943° worst hold); baseline remains 80/81 (2.034°), and Kp=0.10/Kd=0.05 misses once. Even the best margin is small; levels are uncalibrated, so measured rig inertia/damping, sensor noise/bias and delay are needed before selecting gains. |

| 2026-10-16 | Check gain candidate sensitivity to random noise realizations | Added a reproducible 50-seed hold repeat for the critical combined assumed stress corner and included baseline versus Kp=0.14/Kd=0.05 statistics in the report. Added regression checks for pass count and tail behavior. | Both settings pass 49/50; candidate median peak is 1.648° vs 1.682° baseline, but p95 is 1.929° vs 1.885° and worst is 2.112° vs 2.008°. This covers one stress corner only and argues against selecting gains from the one-seed grid; configured gains remain unchanged. |

| 2026-10-16 | Bound the 3000 rpm speed milestone against firmware torque settings | Added an ideal torque/inertia bound to the scenario evaluator and a regression that cross-checks it against the ESP-IDF Kconfig and native-velocity frame source. Updated actuation, hardware, traceability and status notes. | The velocity frame uses the shared torque cap (1 mN·m default, Kconfig maximum 20 mN·m). At the placeholder 1e-4 kg·m² inertia, ideal 20 mN·m yields at most 955 rpm in 0.5 s; the milestone requires 62.8 mN·m. At 20 mN·m, inertia must be ≤3.18e-5 kg·m² before losses. Exact wheel inertia, motor and safe current limits remain unknown; no firmware/hardware performance claim is made. The documented suite and discovery pass 163 tests; `compileall` passes and scenario report regeneration completes successfully. |

| 2026-10-16 | Audit ESP32 Wokwi configuration and verify simulation checks | Added static regression checks for Wokwi firmware/ELF paths, IDF project name, XIAO ESP32-S3 board and octal PSRAM, guest Wi-Fi/private gateway settings, IDF/camera component declaration, and camera CMake inclusion. Added the check module to the documented run command and updated handoff/status/traceability. | The targeted config checks pass; the documented suite and full discovery both pass 165 tests, and `compileall` passes. Static consistency only: no ESP-IDF build or Wokwi boot is available in this environment. |

| 2026-10-16 | Preserve gyro damping when camera tracking is unavailable | Source review found the firmware's controller freshness check rejected missing camera observations even when the IMU remained fresh, contradicting its documented rate-damping fallback. Updated the firmware age policy so a valid visual target requires fresh camera and IMU data, while the no-target path can use fresh IMU data for yaw-rate damping. Added a source-contract regression and corrected camera fallback documentation. | The new source-contract regression, documented suite, and full discovery pass; 166 tests total, `compileall` passes. This does not compile or execute C firmware; ESP-IDF/Wokwi remains unverified. |

| 2026-10-16 | Align host SIL camera-loss behavior with firmware | The Python SIL also coupled IMU rate damping to camera age. Changed observation freshness so camera age gates visual correction only, while gyro damping depends on fresh IMU data; added an SIL integration regression with unavailable camera age and a valid 0.5 rad/s gyro sample. Updated the ICD and CubeSat software notes. | Targeted SIL and C-source contract checks pass. The documented suite and full discovery now pass 167 tests; `compileall` passes. C firmware remains source-checked only and ESP-IDF/Wokwi are unverified. |

| 2026-10-16 | Retain fail-closed behavior for invalid camera freshness | A full-suite run contradicted the attempted camera-age bypass: existing regressions and the persistent handoff require old/future camera samples to fault and command zero torque. Reverted that bypass in C and SIL. The final policy is rate damping only when a fresh valid frame has no target; missing, invalid, stale, or future sensor ages fault closed. Updated firmware/ICD/subsystem/status wording, corrected the camera-init fault log, and kept SIL and C source-contract tests for both branches. | The pre-correction full-suite run exposed one failure (future frame expected `FAULT`, got `RATE_DAMP`). The targeted three-case freshness regression passes; the final documented suite and discovery both pass 167 tests, `compileall` passes. No C build/runtime evidence is available. |

| 2026-10-16 | Guard firmware and SIL telecommand contract consistency | Added a source-level verification that the C firmware and Python SIL expose the same operating-mode names, rejection reasons, and exact wheel-speed/yaw-step argument counts and keys. This complements SIL runtime command tests but does not execute the firmware C parser. | The source-contract check passes. The documented suite and full discovery pass 168 tests, and `compileall` passes. C build/runtime and physical link behavior remain unverified. |

| 2026-10-16 | Preserve camera focal length in operator telemetry logs | Cross-checked the C and SIL telemetry against the Ground Station CSV schema. Added `camera_focal_length_x_px` to the CSV fields and extended the handler logging regression to assert its stored value. Documented that the field accompanies image offsets and onboard bearing error for later calibration review. | The targeted CSV schema/logging tests pass; documented suite and full discovery both pass 168 tests, and `compileall` passes. |

| 2026-10-16 | Enforce onboard telemetry to CSV schema coverage | Added a source-contract regression that extracts the C firmware packet keys and Python SIL result keys, then checks that all non-envelope fields are present in the Ground Station CSV schema. This guards operator data against future silent drops while allowing protocol envelope fields to remain unlogged. | The telemetry schema regression passes; documented suite and full discovery pass 169 tests, and `compileall` passes. This is a static source check and does not execute the C firmware. |


| 2026-10-16 | Evaluate synthetic image-pixel noise before camera detection | Added a deterministic Gaussian-noise evaluator that perturbs rendered QVGA grayscale pixels before running the onboard detector, plus three regressions and a human-readable result report. Detection is 21/21 through assumed σ=40 intensity levels, with errors increasing sharply at σ=40; at σ=50 it is 7/21 and blank-frame false detections are 3/3. Updated camera subsystem, traceability, status report, and handoff to distinguish this from observation-level bearing noise. | Documented suite and full discovery pass 172 tests; `compileall` passes. This synthetic sensitivity screen does not characterize OV5640 noise or model blur, exposure, structured illumination, Bayer processing, or compression. Live networking, ESP-IDF, Wokwi, and hardware remain unverified. |


| 2026-10-16 | Reject isolated threshold noise in Python and firmware detectors | The pixel-level stress screen showed global dark-pixel centroiding could report blank scenes as targets. Added a matched 8-neighbour coherence filter to Python SIL and C firmware, preserving coherent blobs while rejecting isolated threshold crossings. Added Python and portable-C regression cases and regenerated the image-noise study. | Through assumed σ=40, all 21 target samples remain detected with worst centroid change 0.407 px; at σ=50, 18/21 targets remain and 0/3 blank trials produce false detections. Three σ=50 targets are rejected. Focused detector/noise checks pass (11 tests), full discovery passes 173 tests, `compileall` passes, and both model/noise reports were regenerated. A separate 30-call host-only QVGA detector timing sample measured 2.052 ms median / 2.477 ms p95 / 2.980 ms max; it excludes decoding, transport, scheduling, and firmware timing. The C source regression is not compiled because no host C compiler is installed. This is not full component segmentation or camera qualification. |


| 2026-10-16 | Add one-shot portable C host-test runner | Added `CubeSat_ESP32/firmware/run_host_tests.sh` to compile and execute the yaw controller, attitude estimator, and target detector C tests with consistent C11 warning flags, temporary outputs, and cleanup. Documented the command in software and firmware READMEs and the status report. | Shell syntax check passes. Execution reports `Host C compiler not found: cc` and exits 127, confirming this WSL image still cannot run the C checks. No C test result is claimed. |


| 2026-10-16 | Map synthetic camera target size and range acceptance | Expanded the camera geometry sweep to 144 rendered frames across four target diameters, four depths, three horizontal and three vertical bearings using the actual rendered QVGA frame and Python onboard detector. It checks renderer/detector validity parity, X/Y image-error signs, and records raw/coherent dark-pixel counts and centroid errors in `outputs/Verification/Camera_Geometry_Results.md`. Documented the method and limits in camera subsystem and traceability notes; added two regressions and the documented test command. | Geometry evaluator tests pass; the documented module suite and full discovery each pass 176 tests; `compileall` passes and report generation succeeds. This is an ideal synthetic camera-facing disk with provisional focal length/thresholds, not physical camera qualification. |


| 2026-10-16 | Verify rendered target detection across combined body poses | Added five actual QVGA raster cases with a fixed world target and combined yaw/pitch/roll. The Python detector's X/Y offsets match the environment camera metadata at each pose; the vertical comparison explicitly reverses image-row-down coordinates. Added a regression and included the pose grid in the reproducible camera geometry report. | All four camera geometry tests pass; the documented module suite and full discovery each pass 177 tests, `compileall` passes, and the geometry report regenerates. These five low-angle synthetic cases verify renderer/detector consistency only, not camera calibration or physical performance. |


| 2026-10-16 | Increase sample count in synthetic image-noise sensitivity study | Raised deterministic Gaussian pixel-noise sampling from 3 to 30 seeds per bearing (210 target samples and 30 blank frames per noise level). Regenerated the report and updated camera subsystem, traceability, status, and handoff figures. | At assumed σ≤30, 210/210 targets are detected (maximum centroid change 0.457 px); σ=40 is 209/210 (0.769 px maximum), σ=50 is 191/210 (1.153 px maximum), and blank-scene detections are 0/30 at σ=40 and 50. This improves sample coverage but remains synthetic and is not an OV5640 noise specification. The documented module suite and full unittest discovery each pass 177 tests; `compileall` exits successfully with two warnings from vendored `pyparsing` files under `.venv/Lib/site-packages`; Markdown audit finds zero broken relative links across 43 files. |


| 2026-10-16 | Optimize and benchmark Python QVGA detector | Replaced repeated per-pixel neighbour loops with row-bitset 8-neighbour counting while preserving exact raw/coherent counts and centroids. Added deterministic direct-reference differential coverage, dense-scene count coverage, and a reproducible 50-call timing report across clean, noisy, blank, all-dark, checkerboard, and half-dark frames. | Development differential probe matched 300 randomized frames/strides exactly. Targeted camera/noise/geometry tests passed before adding the timing regression. The documented module suite and full discovery each pass 180 tests; targeted `compileall` passes; Markdown audit finds zero broken relative links across 44 files. Regenerated timing report records 5.9 ms median / 10.6 ms maximum for all-dark input in this run. Timing is host Python only, not ESP32 or full-cycle evidence. |


| 2026-10-16 | Reject malformed simulation protocol versions consistently | Offline source review found the C bridge check used only cJSON's integer conversion and could accept fractional versions that truncate to 1. Added an exact `valuedouble == 1.0` C guard, explicit Python numeric/bool validation, a regression over malformed and integral-float versions, a source-contract assertion, and the TM/TC ICD rule. | The documented module suite and full discovery each pass 183 tests; targeted `compileall` passes; Markdown audit checks 44 files with zero broken links. The C source condition remains source-reviewed only: no Linux C compiler/ESP-IDF is installed, and invoking the mounted Windows helper still fails with `UtilBindVsockAnyPort`. |


| 2026-10-16 | Budget and instrument ESP-IDF main-task stack | Official ESP-IDF v6.1 docs identify the main task as the task that runs `app_main()`, set its default stack at 3584 bytes, and document a byte-valued stack high-water API. The synchronous firmware HTTP/control path includes a 2048-byte response body. Set the project main-task stack to 8192 bytes and added a debug-only high-water log every 250 cycles; documented the provisional nature and target measurement gate. | Static Wokwi configuration/source test passes; documented module suite and full discovery each pass 183 tests; `compileall` passes and Markdown audit finds no broken links across 44 files. No ESP-IDF build or runtime stack-margin result is claimed because WSL interop/toolchain is unavailable. |


| 2026-10-16 | Add consolidated offline verification workflow | Added executable `outputs/Python_sim/run_verification.sh` to run full Python discovery and compileall, regenerate the model/camera reports, and conditionally run C host tests and ESP-IDF build when their tools are available. Updated the software README, project README, status report, and handoff. | Full runner execution passes 183 tests, Python compilation, and all five report generators; it prints explicit C11 compiler and ESP-IDF skips in this environment. Shell syntax check passes. It does not run sockets, browsers, Wokwi or hardware. Compiler selection honors `CC`. |

| 2026-10-16 | Prepare first test-drive release | Added `outputs/Python_sim/FIRST_TEST_DRIVE.md`, three Windows WSL batch launchers, and an opt-in PowerShell installer for desktop shortcuts. Updated the software/project READMEs, status report, and handoff. Removed generated bytecode, a Windows-only virtual environment with stale OneDrive metadata, and an old ignored firmware build cache. | `./run_verification.sh` passes 183 Python tests and syntax compilation, regenerates all five reports, and explicitly skips C/ESP-IDF checks because toolchains are absent. Windows launchers and browser behavior were reviewed but not executed because this WSL session cannot invoke Windows binaries or bind loopback sockets. No native `.exe` or live end-to-end pass is claimed. |

| 2026-10-16 | Diagnose first-test-drive connection failure | User reported that neither Windows browser URL, port 8000 nor 8001, could connect. Added WSL/Python preflight checks and persistent diagnostic shell wrappers to the launchers; documented direct WSL startup and PowerShell port checks. | Shell syntax passes for all three WSL wrappers. Windows launch and socket forwarding remain unverified here. Root cause awaits the component terminal errors and `Test-NetConnection` results from the user's desktop. |

| 2026-10-16 | Add one-file WSL/Windows port diagnostic | Added `Launchers/Diagnose_Connections.bat`, a PowerShell runner, and a Python WSL-local probe. It distinguishes service-listener/HTTP failure inside WSL from Windows localhost-forwarding failure and saves a transcript on the Windows desktop. | Python and Bash syntax checks pass; Markdown relative-link audit checks 33 files with zero broken links. The PowerShell script cannot be executed in this environment. No network result is claimed. |

| 2026-10-16 | Keep component terminals open after service exit | User reports launch windows close around browser startup. Wrapped each WSL service command in a persistent Windows `cmd.exe /k` window so errors and the post-exit prompt remain visible; updated the first-test-drive guide and status notes. | Launcher command reviewed, but native Windows `cmd.exe`/WSL execution is unavailable in this session. User-side confirmation is still required. |

| 2026-10-17 | Add full-direction host-SIL pointing and cube reference axes | Following user's Y=0.5 m test, host SIL now controls pitch from vertical image error and roll from gravity-derived leveling using bounded virtual torques. Added centered RGB body frame, Ground Station camera/truth/torque readouts, CSV logging, Y-offset integration regression and full-direction evaluator; updated architecture, API, subsystem, run and handoff docs. Physical firmware remains yaw-only. | The deterministic 10 s (0, 0.5, 2 m) host case reaches <0.11° simulator-truth X/Y errors and <0.03° roll. `./run_verification.sh` passes 187/187 Python tests and syntax compilation and regenerates five reports; C and ESP-IDF steps explicitly skip because toolchains are absent. No live browser, WSL launch, Wokwi or hardware verification is claimed. |

| 2026-10-17 | Speed up simulated roll stabilization | User observed that the body remains tumbling after a Z-axis disturbance longer than it takes to re-point the camera. Raised host-SIL roll PD gains to Kp=0.4 and Kd=0.3 and roll-only virtual torque limit to 0.05 N·m; kept pitch gains/limit unchanged. Added separate bridge limits and deterministic roll pulse recovery/limit regressions, plus updated the test-drive, status report, and API/ICD notes. Added CSV schema rollover that preserves the previous telemetry file intact under a dated legacy name when the header changes. | `./run_verification.sh` passes 190/190 Python tests and syntax compilation and regenerates all five reports; C and ESP-IDF checks skip because toolchains are absent. A 0.05 N·m, 0.5 s Z pulse recovers in about 1.74 s, compared with 3.47 s under the prior tuning. CSV legacy rollover tests pass. This simulation result does not represent a selected physical roll actuator. |

| 2026-10-17 | Prepare second live simulation test drive | Added `outputs/Python_sim/SECOND_TEST_DRIVE.md` with clean startup, baseline, vertical pitch, horizontal yaw, roll-pulse recovery, data capture and interpretation steps. Updated AE/software README entry points, the status report, and the current handoff summary to point to the second-drive checklist and current 190-test checkpoint. | Documentation and link consistency reviewed. The existing 190-test and syntax-compilation result is the latest code verification; no new test run was needed for this documentation-only update. Live Windows/WSL networking and browser behavior remain user-side verification gates. |

| 2026-10-17 | Add yaw IMU damping after camera target loss and moderate stabilization | Added a separate host-SIL lost-target yaw damping gain (0.06 N·m·s/rad, 1.5× tracking D gain) and an end-to-end simulated Y-torque pulse regression that drives the target out of frame and verifies yaw rate falls below 0.02 rad/s. Matched the same lost-target gain in the portable C controller and its host test. Moderated roll PD gains to Kp=0.24/Kd=0.19 and virtual torque limit to ±0.035 N·m; documented the larger allowed roll transient and updated the second test-drive sequence, subsystem/API/ICD notes, traceability, report, and handoff. | `./run_verification.sh` passes 191 Python tests and syntax compilation and regenerates all five reports. The Y pulse (0.1 N·m for 0.5 s) returns rate below 0.02 rad/s within 4.5 s of target loss. The Z pulse (0.05 N·m for 0.5 s) peaks below 11° and reaches the 1°/0.5°/s band within 3 s after pulse. After the C source change, 100 focused Python controller, environment, and firmware-source-contract tests passed. Portable C execution and ESP-IDF checks skipped because toolchains are unavailable. Live browser, Wokwi, firmware, and hardware remain unverified. |

| 2026-10-17 | Create independent NR test-drive snapshots | Created `outputs/Python_sim/NR/Test_drive_1/` and `Test_drive_2/` as self-contained copies of the AE project, each with its own source, inputs, reports/outputs, README and context. Test_drive_1 preserves the 190-test first full-direction tuning; Test_drive_2 preserves the current 191-test yaw-damping and moderated-roll tuning. Added a snapshot index, linked it from the live software README, and recorded the handoff. | Confirmed both copies contain all three runnable components, project inputs and context handoff; checked Markdown links (zero broken after correcting two first-drive links) and removed Windows `Zone.Identifier` sidecars. Snapshots exclude `.git`, generated telemetry logs, caches and build outputs. No code tests were run for this file-copy/documentation task; prior verification counts describe the source states at capture. |

| 2026-10-17 | Resume AE goal and verify all simulated control axes | Added deterministic host-SIL axis checks using 0.5 s disturbances at half the provisional pitch/yaw/roll torque limits while the camera is occluded. Added independent camera-centering checks for yaw/image X and pitch/image Y, plus a point-target roll-observability check. Integrated the new report and three regression tests into the verification runner; updated traceability and camera/control/design notes. | Focused `Verification.test_axis_control` passes 3/3. Report measurements: pitch settles to gravity-level <1°/<0.5°/s 2.01 s after pulse; yaw rate settles <0.5°/s in 0.60 s but retains the new 2.64° heading; roll levels in 1.32 s. Camera cases settle under 2° in 2.34 s (yaw) and 1.03 s (pitch). A centered featureless point supplies zero X/Y error at 15° camera roll, so camera-directed roll is not observable with this detector; gravity-derived IMU torque levels it. `./run_verification.sh` then passes 194 Python tests, compileall and all report generators; portable C and ESP-IDF checks skip because the toolchains are absent. Live sockets, firmware, emulator and hardware remain unverified. |

| 2026-10-17 | Add camera-directed roll to the simulated CubeSat pipeline | Added an optional asymmetric three-dot target fiducial (disabled by default), directed image-angle estimation in Python and portable C, camera-based roll feedback with IMU fallback, and a bounded C pitch/roll controller that supplies virtual torques only for simulator-framed ESP32 telemetry. Added a 24°/20° inner-field limit to keep the fiducial un-clipped during edge acquisition. Exposed marker toggle, measurement/source telemetry, Ground Station CSV/UI fields, host and C test cases, architecture/API/ICD/traceability/status updates, and the second test-drive roll procedure. | `./run_verification.sh` passes 199 Python tests, `compileall`, and all six reports. The 15° camera-roll case settles within ±1°/±0.5°/s in 8.32 simulated seconds with accelerometer roll hidden; yaw/pitch camera pointing remains below 2° with the marker enabled. The C module test is included in the runner but skipped because no C11 compiler is available; ESP-IDF/Wokwi, sockets/browser, physical target marker, and hardware remain unverified. Point-only mode remains default and uses IMU roll fallback. |
# 2026-10-17 — Wokwi configuration and setup review

Rechecked the ESP-IDF/Wokwi configuration against the official Wokwi ESP-IDF,
project configuration, and Wi-Fi gateway documentation. Confirmed static
agreement between `wokwi.toml`, CMake project name, ESP32-S3 target, XIAO
ESP32-S3 board, and octal PSRAM setting. Expanded the firmware README with the
build/workspace/startup sequence, private-gateway requirement for access to the
local plant, and the ESP-IDF 6.0+ requirement for `idf.py wokwi`. Corrected a
stale README description from raw frame transfer to the implemented RLE1 path.
Added these sourced emulator constraints to `references/Hardware_and_Emulation_Notes.md`,
the operator-facing `outputs/reports/AE_Project_Status_and_Run_Guide.md`, and
the continuation gate to `Context/HANDOFF.md`.

No firmware compiler, ESP-IDF build, Wokwi runtime, or live bridge check was
available; no runtime verification is claimed. Sources: [Wokwi ESP-IDF guide](https://docs.wokwi.com/vscode/esp-idf),
[project configuration](https://docs.wokwi.com/vscode/project-config), and
[Wi-Fi gateway guide](https://docs.wokwi.com/guides/esp32-wifi).

## 2026-10-17 — Expand portable C camera-roll coverage

Expanded `CubeSat_ESP32/firmware/test_black_target.c` from a single positive
fiducial angle to a five-angle signed sweep from -30° through +30°, checking
orientation error and centroid centering. The existing host-test runner
includes this test. No C compiler is available in this WSL session, so the
changed C test has not been compiled or run; this is additional regression
source coverage only, not verification evidence.

In the same review, changed `cubesat_main.c` to require an exact
`camera_frame_source == "simulated"` before enabling virtual pitch/roll plant
torques; the previous first-character check was broader than intended. Extended
`Verification/test_firmware_source_contracts.py` to guard that gate, freshness
and fault checks, and camera-roll preference with IMU fallback. The focused
axis/control/source-contract/emulation suite passes 10/10; C firmware remains
uncompiled. Updated the CAM-02 traceability row to distinguish the new C test
source coverage from executed verification. Then reran `./run_verification.sh`:
199/199 Python tests, syntax compilation, and all six report generators passed;
C and ESP-IDF were explicitly skipped because `cc` and `idf.py` are absent.

## 2026-10-17 — Verify simulated pitch/roll sensor axes at combined poses

Strengthened the existing five-pose environment/SIL regression to assert that
accelerometer-derived pitch and roll estimates recover plant truth across
combined yaw, pitch and roll, in addition to checking gravity magnitude and
yaw estimation. Targeted test passes; the full verification runner passes
199/199 Python tests, syntax compilation and all six report generators. C and
ESP-IDF checks remain skipped. This confirms only the ideal synthetic sensor
frame mapping, not physical mounting or dynamic accelerometer effects.

## 2026-10-17 — Verify exact simulator frame-source gate at the plant bridge

Expanded the environment bridge regression to accept ESP32 virtual-axis torque
only for the exact `simulated` camera-frame label and reject `physical`,
`simulated-extra`, `unavailable`, and missing labels. Targeted bridge/firmware
contract checks pass 5/5. The full verification runner was rerun after this
change: 199 Python tests, syntax compilation, and all six report generators
pass; portable C and ESP-IDF are still skipped due unavailable tools.

## 2026-10-17 — Bring progress estimate and project tree current

Audited the status report against the preserved request to show a simulated-work
estimate and a compact project tree mapped to `inputs/Prompts.txt`. Retained the
rough 65% estimate and made its qualitative meaning/remaining runtime gates
explicit. Rebuilt the tree to include all named subsystems and hardware while
separating implemented, Python-tested, source-only, and unverified items.
Marked the older launcher-preparation text as a historical checkpoint. No code
or runtime evidence changed in this report-only update.

## 2026-10-17 — Bound portable C image dimensions

Source review identified that the generic C RLE1 decoder accepted an arbitrary
requested output pixel count and the target detector accepted arbitrarily
large row strides, despite fixed supported image maxima. Added a 640×480 decoded
raster limit and a 640-byte stride limit, plus C test cases for oversized pixel
requests/no partial write and oversized stride. Added Python source-contract
checks and updated CAM-01 traceability/status notes. Corrected the CubeSat
README to distinguish the generic raster cap from the active QVGA requirement.
Focused checks pass and the full runner passes 199 Python tests, syntax
compilation, and all six report generators; C tests/ESP-IDF remain skipped
because the required tools are absent.



## 2026-10-17 — Validate C camera buffer length

Extended the portable C detector API with the actual grayscale buffer length
and a `stride * height` minimum-size guard. Updated the simulator frame and
physical camera adapter to pass their known buffer lengths; added a truncated
frame case to the C test source and a Python source-contract check. Focused
source-contract tests pass 4/4. The full runner passes 199 Python tests,
syntax compilation, and all six report generators. C host tests and ESP-IDF
remain skipped because `cc` and `idf.py` are unavailable, so the C regression
and firmware build have not executed. Updated the Camera subsystem and CubeSat
README with the frame-buffer contract and test limitation; a six-file targeted
Markdown audit found no broken relative links or carriage returns.


## 2026-10-17 — Check C detector API call consistency

Added a Python source-contract regression that scans all firmware C/header
files and confirms every detector declaration/call uses seven arguments after
adding the accessible-buffer-length parameter. Focused source contracts pass
5/5; the full runner passes 200 Python tests, syntax compilation, and all six
reports. Portable C and ESP-IDF remain skipped because no C compiler or
`idf.py` is installed. Checked common compiler paths and Python C-parser
packages; none were available. This source check verifies interface consistency,
not C syntax or runtime behavior.


## 2026-10-17 — Wait for service readiness in Windows launchers

Replaced fixed browser-open delays in the environment and Ground Station
launchers with a PowerShell HTTP readiness poll (45-second timeout, status 200
and expected page title). Failure stays visible; service terminals continue in
`cmd.exe /k`. Added static contract coverage and documented Windows-runtime
limits in the test-drive/readme/status/handoff material. Focused launcher config
tests pass 3/3, all wrapper scripts pass `bash -n`, and full verification passes
201 Python tests, syntax compilation, and all six reports. Windows batch and
PowerShell launch behavior is not executable in this WSL session; C/ESP-IDF
remain skipped for unavailable tools.


## 2026-10-17 — Make host SIL bridge startup resilient

Changed `CubeSat_ESP32.sil` to report waiting rather than claiming an early
connection, retry connection-reset and HTTP protocol failures with bounded
50–500 ms exponential backoff, rate-limit repeated failures to five-second
intervals, and announce a connection/recovery only after a complete successful
cycle. Added tests for the
retry curve and recovery after an initial bridge error, and documented expected
terminal output. Focused retry/launcher/firmware-contract checks pass 10/10; the
full runner passes 203 Python tests, syntax compilation, and all six reports. C
and ESP-IDF remain skipped due unavailable tools; no live sockets or Windows
launcher behavior is claimed.


## 2026-10-17 — Correct firmware camera pitch direction

Compared detector and host-SIL coordinate conversions and found C firmware was
negating the detector's positive-down Y offset, opposite to the host controller
for camera-guided pitch. Removed the extra sign inversion and added a source
contract asserting the bearing conversion; documented raster-versus-scene Y
conventions in the camera/control subsystem notes, architecture, ICD and
traceability. Focused source-contract plus control tests pass 44/44. Full
verification passes 203 Python tests, syntax compilation, and all six reports.
Portable C and ESP-IDF remain uncompiled because their toolchains are absent.


## 2026-10-17 — Check C/SIL accelerometer tilt parity

Compared firmware and host-SIL pitch/roll gravity formulas with the simulated
accelerometer frame. Both derive pitch with `atan2(-az, hypot(ax, ay))`, roll
with `atan2(ax, ay)`, and accept 0.8–1.2 g magnitude. Added source-contract
assertions and recorded the limitation that the C path and physical sensor
mounting are unverified. Focused source contracts pass 5/5; the existing full
runner result remains 203 tests passing, `compileall`, and six reports.


## 2026-10-17 — Guard C/Python pitch-roll controller parity

Added a source-contract regression comparing all portable C pitch/roll default
gains, limits and validity settings with the Python reference, plus normalized
pitch and roll torque equations. Focused firmware source contracts pass 6/6.
The full runner passes 204 Python tests, syntax compilation, and all six
reports. C and ESP-IDF remain unbuilt because the toolchains are not installed.
A parallel source review found the C and Python marker covariance/third-moment
formulas, direction logic, thresholds and angle wrapping equivalent; existing C
test source sweeps signed marker angles, but has not been executed.


## 2026-10-17 — Guard yaw controller parity

Added a firmware source-contract regression that compares all six C yaw
controller defaults with the Python SIL defaults and verifies the tracking and
lost-target rate-damping branches match. The focused firmware contract suite
passes 7/7; the full runner passes 205 Python tests, syntax compilation and all
six reports. Portable C and ESP-IDF still skip for missing compiler/tools, so
this establishes source parity rather than executed firmware behavior. Added
the SIL startup-retry test module to the focused unittest commands in both
project READMEs.


## 2026-10-17 — Align yaw-step terminal status

Found a C/Python boundary mismatch when attitude became invalid on the exact
yaw-step deadline: C reported `ATTITUDE_INVALID`, Python `TIMEOUT`. Both safely
entered SAFE, but their telemetry differed. Reordered the Python checks to
match C and added a simultaneous-condition regression plus source-contract
checks for timeout, settle-band and dwell limits. Focused checks pass 9/9; full
verification passes 207 Python tests, syntax compilation and all six reports.
C/ESP-IDF remain uncompiled because the toolchains are unavailable.
| 2026-10-17 | Ordered telecommand queue regression | Added a bridge test that queues two commands and proves only the oldest is offered per onboard cycle, with the second delivered after the controller advances its command ID. The full verification runner passes 208 Python tests, syntax compilation, and all six reports; C/ESP-IDF skip for unavailable toolchains, and live sockets remain unverified. |
| 2026-10-17 | Verify combined camera correction on yaw, pitch and roll | Added an integrated host-SIL trajectory from 10° yaw, −8° pitch, and 12° roll with an off-axis target and asymmetric marker. Simultaneous image centering and marker roll correction stay in the specified simulation bands for 0.5 s starting at 1.99 s. Focused axis checks pass 5/5; full verification passes 209 Python tests, syntax compilation and all six reports. Firmware C/ESP-IDF and live hardware remain unverified. |
| 2026-10-17 | Verify ESP32 virtual-axis telemetry routing | Added a source contract that checks firmware telemetry field names and simulator-frame gating against the bridge, and extended the bridge test to confirm accepted simulated pitch/roll torques drive the matching plant axes and telemetry outputs. Focused checks pass; full verification passes 210 Python tests, syntax compilation and all six reports. C/ESP-IDF, Wokwi and live HTTP remain unverified. |
| 2026-10-17 | Clarify firmware/emulator run instructions | Audited the available toolchain commands and corrected duplicated verification-script wording and the ESP-IDF README description of simulator-only pitch/roll torque. Current PATH has Python but no C compiler, `idf.py`, Wokwi CLI, Node.js or npm; the full offline run still passes 210 Python tests and syntax compilation, with C/ESP-IDF explicitly skipped. |
| 2026-10-17 | Improve local HTTP startup diagnostics | After confirming the environment and Ground Station both fail here with `PermissionError: Operation not permitted`, added shared diagnostics for denied socket creation and occupied ports. Both services now bind before starting background work where relevant. Four tests cover the helper and both service integrations. The full runner passes 214 Python tests, syntax compilation and all six reports; the local sandbox still prevents live TCP verification. |
| 2026-10-17 | Add OV5640 autofocus startup validation | Compared Seeed's XIAO Sense camera guidance with Espressif `esp32-camera` 2.1.8 autofocus API. Added PID-based OV5640 autofocus initialization and explicit focused-status check; failure deinitializes the camera. Fixed-focus variants remain supported. Updated component minimum, conditional CMake dependency, SDK defaults, source-contract coverage and camera/hardware docs. Focused checks pass 12/12; full suite passes 214/214 and syntax compilation. Five reports regenerated by the runner and the sixth rerun successfully. Firmware build and hardware remain unverified because the required toolchain/board are unavailable. |
| 2026-10-17 | Fix autofocus feature-header include order | Source review found `CONFIG_CAMERA_AF_SUPPORT` was checked before `sdkconfig.h` inclusion. Reordered includes and added a source-contract regression. Focused source/config tests pass 12/12; syntax compilation and updated-document local-link check pass. C/ESP-IDF compilation remains unavailable. |
| 2026-10-17 | Verify simultaneous sensor stabilization on XYZ | Added a host-SIL case applying half-limit pitch, yaw, and roll torques together during camera occlusion. All three controller outputs activate and the simultaneous rate/tilt bands settle 2.01 s after the pulse. Axis suite passes 6/6, full Python suite 215/215, syntax compilation passes, and the axis report was regenerated. This is decoupled host-plant evidence; C/ESP-IDF, Wokwi, live network and hardware remain unverified. |
| 2026-10-17 | Restrict physical camera startup to verified module variants | Changed the XIAO adapter to accept OV5640 only after successful autofocus and OV3660 as fixed focus; unknown camera PIDs now deinitialize and fail startup. Updated source contracts, firmware/camera guidance, hardware note, and traceability. This matches Seeed's documented transition pair; firmware compilation and physical PID detection remain unverified. |
| 2026-10-17 | Add selectable LIS3MDL full-scale range | Checked ST AN4602 and datasheet FS bits and sensitivities. Added a Kconfig choice for ±4/8/12/16 gauss, retained ±4 default, configured CTRL_REG2 accordingly, and scaled raw samples with the matching LSB/gauss value. Updated firmware/subsystem/reference/status documentation and added static config/source coverage. Focused checks pass 13/13; all 216 Python tests and syntax compilation pass. ESP-IDF build and physical breakout remain unverified; SPI is not implemented. |


### Report LIS3MDL full-scale setting in telemetry

Propagated the selected physical LIS3MDL range through firmware telemetry and
Ground Station UI/CSV; synthetic SIL reports null. Updated API/ICD/subsystem
docs and project status. Focused checks pass 73/73, full Python suite 217/217,
and compileall passes. Firmware C/ESP-IDF and physical behavior remain
unverified.


### Refresh simulation results and verify camera roll in both directions

The full runner passed 217 tests and syntax compilation, then stopped in report
generation. Reran the simulation evaluator directly, replacing the temporary
historical snapshot with a fresh 17 October report. Extended the all-axis
camera-roll check to +15° and −15° with accelerometer roll hidden; the axis
suite passes 7/7. The bilateral axis report is complete. Full test discovery later confirmed
218 passing tests.


### Verify oriented camera-roll correction in both directions

Parameterized the host-SIL oriented-marker roll case and added +15°/−15°
coverage with accelerometer roll hidden. Both signs select CAMERA, command
corrective torque, and settle under 1° and 0.5°/s. Regenerated the axis report
with both results and regenerated the current simulation model report. Focused
axis tests pass 7/7; full Python discovery passes 218/218; compileall passes.
Firmware C/ESP-IDF, Wokwi, live browser/TCP, and physical behavior remain
unverified.

### Correct setup guide's simulation-axis description

The setup guide inaccurately said only yaw was actively controlled. Updated it
to describe all-axis host SIL, simulator-framed ESP32 virtual pitch/roll, and
the yaw-only physical motor path. Documentation-only correction; the verified
218 Python tests and syntax compilation remain current.

### Remove duplicate work from the bilateral camera-roll test fixture

Changed the axis test class to reuse its +15° result from the bilateral sweep
instead of running that simulation twice. Focused axis verification passes 7/7
in 48.12 s. `compileall` and an updated-file whitespace/content check pass. No
C compiler, ESP-IDF, or Wokwi CLI is installed/on PATH, so those firmware gates
remain open.


### Verify camera yaw and pitch correction from both image-error signs

Expanded the host-SIL camera-pointing evaluator from one positive-side target
per axis to signed target offsets on both yaw/image-X and pitch/image-Y. All four
rendered-frame cases settle under 2° and assert opposite signed initial errors
with cross-axis projection error <1e-5 rad. The focused axis suite passes 7/7; full discovery passes 218/218 in 137.37 s,
and `compileall` passes. The refreshed report shows 1000/1000 valid frames per
case and 2.29–2.34 s yaw / 1.03 s pitch settling.

### Close out signed camera-axis evidence and check firmware toolchain locations

Full Python discovery passes 218/218 in 137.37 s after adding ±yaw/image-X and
±pitch/image-Y rendered-frame cases; the axis report now lists signed initial
errors and settling times. `compileall` and local Markdown-link checks pass.
A file search under `/home/nunot`, `/opt`, and `/usr/local` found no IDF,
ESP32-S3 compiler, GCC, or Clang, confirming firmware compilation cannot run in
this environment. No C/ESP-IDF runtime claim is made.

### Align portable C test instructions with the actual runner

Compared `run_host_tests.sh` with both READMEs. The runner includes four suites,
including the pitch/roll controller, while the docs said three. Updated the
firmware and Python_sim README text. The C tests remain unexecuted because a
host compiler is unavailable.


### Document available ESP-IDF setup routes

Confirmed the Windows ESP-IDF 6.1 tree at `C:\esp\v6.1\esp-idf` and its
Xtensa toolchain under `C:\Espressif\tools`. Direct WSL execution fails with
`UtilBindVsockAnyPort`; no native Linux compiler/IDF is installed. Consulted
Espressif's official v5.3.5 setup and Wokwi ESP-IDF docs, then added versioned
WSL install commands plus a Windows PowerShell build attempt using the current
WSL UNC project path. The Windows route remains unverified.

### Probe current local runtime visibility

Checked the sandbox process list for all three service entry points and probed
ports 8000/8001 with curl. No matching process was visible in the sandbox and
both requests failed while opening a socket (`Operation not permitted`), before
HTTP. This cannot establish state in a separate user desktop session or expose
its live telemetry.

### Verify current Python prerequisite and shared firmware outputs

The active WSL interpreter reports Python 3.14.4; the official ESP-IDF v5.3.5
Linux guide specifies Python 3.8 or newer. Checked the current shared firmware
build path: `flasher_args.json`, `cube_firmware.elf`, and
`cube_firmware.bin` are all absent, so no build artifact has appeared here.

### Guard Wokwi XIAO ESP32-S3 memory attributes

Checked the existing diagram's 8 MB flash and 8 MB octal PSRAM values against
Wokwi's official XIAO ESP32-S3/ESP32 configuration documentation. Extended
the static emulation test to assert flash and PSRAM sizes remain within the
documented model choices. The focused emulation and firmware source-contract
suites pass 14/14. This does not build or boot firmware; the native C/ESP-IDF
toolchain and Wokwi runtime remain unavailable. Reference: [Wokwi ESP32
simulation guide](https://docs.wokwi.com/guides/esp32).

### Refresh complete offline verification run

Reran `./run_verification.sh` from the current Python_sim project. Full
discovery passed 218/218 in 137 s, Python syntax compilation passed, and all
six simulation/camera/axis report generators completed. Portable C and
ESP-IDF stages reported explicit skips because their tools are missing.
Separately reran `Verification.test_axis_control` (7/7 pass). Updated the
operator report and handoff to replace the stale prior-run interruption note.
No live sockets, browser, C binary, ESP-IDF firmware, Wokwi boot, or hardware
was exercised.

### Recheck Windows ESP-IDF access from WSL

Confirmed `cc`, `gcc`, `clang`, `idf.py`, and `wokwi-cli` remain absent from the
WSL PATH. The installed Windows `clang.exe` and `cmd.exe` both fail to start
with `UtilBindVsockAnyPort`, so the ESP-IDF Windows cross-compiler remains
inaccessible to this process. The native Windows PowerShell build route is
already documented in the firmware README; this run did not change it or
produce build artifacts.

### Recheck live loopback capability

Attempted to create/listen on the local simulation and Ground Station ports
8000 and 8001. Socket creation was denied for both by the sandbox with
`PermissionError: Operation not permitted`, before either service could start.
This revalidates that the missing browser/API trial is an environment gate,
not a diagnosed application connection failure.

### Reconcile requirement and status reports with latest runner

Audited the subsystem deliverable tree and found all seven expected
descriptions. Updated stale completion notes in requirements traceability and
the status report: the current full runner completed 218 Python tests, syntax
compilation, and six report generators; C and ESP-IDF remain explicit tool
skips. The socket-denied and Windows-interop limits are stated separately so
they are not presented as application test results.

### Confirm recorded publication of the NR snapshots

Inspected the Python_sim Git repository after the user previously asked to
upload both test-drive snapshots. Commit
`e398cb19797eb065a299ce8f8222beeab96e1579` contains 125 tracked files for
Test_drive_1 and 126 for Test_drive_2. The local Git reflog says
`origin/main` was updated by push at 2026-10-17 01:16 +0100; `main` and the
cached upstream ref are equal at that commit. A fresh `git push origin main`
could not resolve `github.com`, so the current remote could not be rechecked.
Uncommitted live workspace changes are outside this snapshot commit.
