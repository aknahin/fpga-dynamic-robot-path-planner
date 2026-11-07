"""Mac side of the UDP link to the FPGA (or to bridge/mock_fpga.py).

Sends one frame as 4 C3 camera strips (front, left, back, right) plus a C2
pose, then waits for the B6 reply with the same frame number. It only moves
bytes; the decisions are in the reply.
"""

import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ref'))
import packets_ff as P  # noqa: E402

REPLY_TIMEOUT_S = 0.2


class UdpLink:
    def __init__(self, target):
        """target: 127.0.0.1 for the mock on this Mac, 192.168.10.255 for the board."""
        self.target = (target, P.CMD_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):  # a viewer can listen on 5005 too
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', P.STATE_PORT))
        self.frame = 0
        self.timeouts = 0

    def reset(self):
        self.sock.sendto(P.pack_reset(), self.target)

    def exchange(self, images, pose):
        """Send one frame; return the parsed B6 reply, or None on timeout.

        images: 4 cameras of 2 x 32 depth bytes (see ref/warehouse.py)."""
        self.frame = (self.frame + 1) & 0xFF
        self._drain()
        for payload in P.pack_cameras(self.frame, images):
            self.sock.sendto(payload, self.target)
        self.sock.sendto(P.pack_pose(self.frame, *pose), self.target)

        deadline = time.monotonic() + REPLY_TIMEOUT_S
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                self.timeouts += 1
                return None
            self.sock.settimeout(left)
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            reply = P.parse_reply(data)
            if reply and reply['frame'] == self.frame:
                return reply

    def _drain(self):
        """Drop old replies and heartbeats queued since the last frame."""
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(2048)
        except (BlockingIOError, socket.error):
            pass
        self.sock.setblocking(True)
