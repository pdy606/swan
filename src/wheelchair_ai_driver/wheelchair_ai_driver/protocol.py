"""Bounded JSON framing for the ROS/isolated-worker local socket pair."""
import json
import struct

MAX_MESSAGE_BYTES = 32 * 1024 * 1024


def send_message(connection, message):
    payload = json.dumps(message, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ValueError('Inference message exceeds size limit')
    connection.sendall(struct.pack('!I', len(payload)) + payload)


def _read_exact(connection, count):
    result = bytearray()
    while len(result) < count:
        chunk = connection.recv(count - len(result))
        if not chunk:
            raise EOFError('Inference worker connection closed')
        result.extend(chunk)
    return result


def receive_message(connection):
    length = struct.unpack('!I', _read_exact(connection, 4))[0]
    if length > MAX_MESSAGE_BYTES:
        raise ValueError('Inference message exceeds size limit')
    return json.loads(_read_exact(connection, length))
