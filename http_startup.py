"""Consistent, actionable startup errors for local HTTP services."""

import errno
import sys


def open_http_server(server_type, address, handler, service_name):
    """Bind a server or exit with a concise local-port diagnostic."""
    host, port = address
    try:
        return server_type(address, handler)
    except PermissionError as exc:
        print(
            f"{service_name} cannot listen on {host}:{port}: {exc}. "
            "The operating system denied local socket creation; check that "
            "this session permits local TCP servers.",
            file=sys.stderr,
        )
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            reason = "the address is already in use; stop the existing service"
        else:
            reason = "check local network permissions and the requested address"
        print(
            f"{service_name} cannot listen on {host}:{port}: {exc}; {reason}.",
            file=sys.stderr,
        )
    raise SystemExit(1) from None
