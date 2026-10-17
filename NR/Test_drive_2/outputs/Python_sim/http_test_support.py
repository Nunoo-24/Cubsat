"""Small in-memory HTTP request runner for handler tests without TCP sockets."""

from io import BytesIO
import json


class _MemorySocket:
    def __init__(self, request_bytes):
        self._input = BytesIO(request_bytes)
        self._output = BytesIO()

    def makefile(self, mode, buffering=-1):
        return self._input if "r" in mode else self._output

    def sendall(self, data):
        self._output.write(data)


def dispatch(handler_class, method, target, payload=None):
    if payload is None:
        body = b""
    elif isinstance(payload, bytes):
        body = payload
    else:
        body = json.dumps(payload).encode("utf-8")
    headers = ["Host: localhost", "Connection: close"]
    if body:
        headers.extend(("Content-Type: application/json",
                        f"Content-Length: {len(body)}"))
    request = (f"{method} {target} HTTP/1.0\r\n" +
               "\r\n".join(headers) + "\r\n\r\n").encode("ascii") + body
    memory_socket = _MemorySocket(request)
    handler_class(memory_socket, ("127.0.0.1", 0), object())
    response = memory_socket._output.getvalue()
    header, response_body = response.split(b"\r\n\r\n", 1)
    status = int(header.split(b"\r\n", 1)[0].split()[1])
    return status, header, response_body
