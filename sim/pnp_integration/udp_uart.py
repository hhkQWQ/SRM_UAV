"""Datagram-backed any/read/write adapter for the original FlightLink."""
import socket


class UdpUart:
    def __init__(self, port=14540):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', int(port)))
        self.socket.setblocking(False)
        self.buffer = bytearray()
        self.peer = None

    def _receive(self):
        for _ in range(32):
            try:
                packet, peer = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if peer[0] != '127.0.0.1':
                continue
            if self.peer is None:
                self.peer = peer
            if peer != self.peer:
                continue
            self.buffer.extend(packet)
            if len(self.buffer) > 65536:
                self.buffer.clear()

    def any(self):
        self._receive()
        return len(self.buffer)

    def read(self, size):
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def write(self, data):
        if self.peer is None:
            return 0
        try:
            return self.socket.sendto(data, self.peer)
        except BlockingIOError:
            return 0

    def close(self):
        self.socket.close()
