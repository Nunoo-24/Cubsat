# TMTC Interface Control Document

## Status

Level 0 provisional interface.  
Used for local Ground Station and simulated Cube integration.

## Transport

- Protocol: UDP
- Ground Station address: `127.0.0.1`
- Ground Station receive port: `5005`
- Encoding: UTF-8 JSON
- Update rate: 2 Hz during the initial simulator test

> This transport is provisional and will be adapted when the official simulator
> interface is supplied.

## Field definition

| Field | Type | Description |
|---|---|---|
| `sequence` | integer | Increments for every telemetry packet |
| `timestamp` | string | UTC timestamp in ISO 8601 format |
| `mode` | string | Current operating mode |
| `system_state` | string | Overall system health/state |
| `alive` | boolean | Heartbeat indicating that the Cube/simulator is running |

## Telemetry packet

```json
{
  "sequence": 1,
  "timestamp": "2026-10-06T10:00:00+00:00",
  "mode": "SIMULATION",
  "system_state": "NOMINAL",
  "alive": true
}