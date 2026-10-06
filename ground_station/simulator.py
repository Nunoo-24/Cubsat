import json
import socket
import time
from datetime import datetime, timezone


HOST = "127.0.0.1"
PORT = 5005

socket_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sequence = 0

print(f"Sending simulated telemetry to {HOST}:{PORT}")
print("Stop with Ctrl + C")

try:
    while True:
        sequence += 1

        telemetry = {
            "sequence": sequence,
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mode": "SIMULATION",
            "system_state": "NOMINAL",
            "alive": True,
        }

        socket_out.sendto(
            json.dumps(telemetry).encode("utf-8"),
            (HOST, PORT),
        )

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nSimulator stopped.")
    