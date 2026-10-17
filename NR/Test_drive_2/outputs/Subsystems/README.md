# AE subsystem design notes

This index maps the requested system breakdown to implementation and interface documents. Code remains in `Python_sim/` so the three runnable programs stay together; each subsystem note explains its boundary and current state without copying source code.

| Subsystem note | Main implementation |
|---|---|
| [Simulation environment](Simulation_Environment/README.md) | `../Python_sim/Sim_Environment/` |
| [Camera and visual target sensing](Camera/README.md) | Python reference and firmware detector/adapter |
| [IMU and attitude sensing](Sensor/README.md) | Firmware MPU-6050 adapter and environment model |
| [Onboard control](Control/README.md) | Python reference and C yaw controller |
| [Reaction-wheel actuation](Actuation/README.md) | Compile-time disabled moteus UART adapter; not yet compiled or hardware-tested |
| [Telemetry and telecommand](Telemetry/README.md) | Environment bridge and Ground Station proxy |
| [Ground Station](Ground_Station/README.md) | `../Python_sim/Ground_Station/` |

For shared signal flow, schemas and model boundaries, see [system architecture](../Python_sim/docs/System_Architecture_and_Interfaces.md), [simulation API](../Python_sim/docs/Simulator_HTTP_API.md), and [TM/TC ICD](../Python_sim/docs/TMTC-ICD.md).
