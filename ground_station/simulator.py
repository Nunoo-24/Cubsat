import json
import socket
import time
from datetime import datetime, timezone

TELEMETRY_HOST = "127.0.0.1"
TELEMETRY_PORT = 5005
COMMAND_HOST = "127.0.0.1"
COMMAND_PORT = 5006

telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
command_socket.bind((COMMAND_HOST, COMMAND_PORT))
command_socket.setblocking(False)

sequence = 0
last_command = "—"
command_status = "—"

print(f"Sending telemetry to {TELEMETRY_HOST}:{TELEMETRY_PORT}")
print(f"Listening for commands on {COMMAND_HOST}:{COMMAND_PORT}")
print("Stop with Ctrl + C")

try:
    while True:
        try:
            data, _ = command_socket.recvfrom(1024)
            command = json.loads(data.decode("utf-8"))

            if command.get("command") == "PING":
                command_id = command.get("command_id", "unknown")
                last_command = f"PING #{command_id}"
                command_status = "ACKNOWLEDGED"
                print(f"Received and acknowledged {last_command}")
            else:
                last_command = str(command.get("command", "UNKNOWN"))
                command_status = "REJECTED"

        except BlockingIOError:
            pass
        except (UnicodeDecodeError, json.JSONDecodeError):
            command_status = "INVALID COMMAND"

        sequence += 1

        telemetry = {
            "sequence": sequence,
            "timestamp": datetime.now(timezone.utc).isoformat(
                timespec="seconds"
            ),
            "mode": "SIMULATION",
            "system_state": "NOMINAL",
            "alive": True,
            "last_command": last_command,
            "command_status": command_status,
        }

        telemetry_socket.sendto(
            json.dumps(telemetry).encode("utf-8"),
            (TELEMETRY_HOST, TELEMETRY_PORT),
        )

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nSimulator stopped.")