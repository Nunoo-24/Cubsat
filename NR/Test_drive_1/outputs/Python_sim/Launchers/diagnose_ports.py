"""Print WSL-local listener and HTTP checks for the two browser ports."""

import socket
import subprocess
import sys
from urllib.error import URLError
from urllib.request import urlopen


PORTS = (8000, 8001)

print(f"WSL Python: {sys.version.split()[0]} ({sys.executable})")
print("WSL-local TCP and HTTP checks:")
for port in PORTS:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=2):
            print(f"  port {port}: TCP CONNECTED")
    except OSError as exc:
        print(f"  port {port}: TCP FAILED: {type(exc).__name__}: {exc}")
        continue

    try:
        with urlopen(f"http://127.0.0.1:{port}/", timeout=3) as response:
            print(f"  port {port}: HTTP {response.status}")
    except (OSError, URLError) as exc:
        print(f"  port {port}: HTTP FAILED: {type(exc).__name__}: {exc}")

print("WSL listener table (ss -ltn):")
try:
    result = subprocess.run(
        ["ss", "-ltn"], capture_output=True, text=True, check=False, timeout=3
    )
    lines = [line for line in result.stdout.splitlines()
             if ":8000" in line or ":8001" in line]
    print("\n".join(lines) if lines else "  no listeners shown on 8000/8001")
    if result.returncode:
        print(result.stderr.strip() or f"  ss exited with {result.returncode}")
except (OSError, subprocess.TimeoutExpired) as exc:
    print(f"  unable to run ss: {exc}")
