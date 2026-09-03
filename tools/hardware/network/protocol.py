"""Wire-compatible length-prefixed JSON protocol for upper/lower control."""

from __future__ import annotations

import json
import socket
import struct
from typing import Any

HEADER = struct.Struct("!I")
MAX_PAYLOAD = 64 * 1024


class ProtocolError(RuntimeError):
    pass


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise EOFError("peer closed connection")
        data.extend(chunk)
    return bytes(data)


def encode_frame(message: dict[str, Any]) -> bytes:
    payload = json.dumps(message, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ProtocolError(f"payload must be 1..{MAX_PAYLOAD} bytes")
    return HEADER.pack(len(payload)) + payload


def send_frame(sock: socket.socket, message: dict[str, Any]) -> None:
    sock.sendall(encode_frame(message))


def recv_frame(sock: socket.socket) -> dict[str, Any]:
    (size,) = HEADER.unpack(recv_exact(sock, HEADER.size))
    if not 1 <= size <= MAX_PAYLOAD:
        raise ProtocolError(f"invalid payload size: {size}")
    try:
        value = json.loads(recv_exact(sock, size).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("message must be a JSON object")
    return value
