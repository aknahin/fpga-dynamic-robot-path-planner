#!/usr/bin/env python3
"""Mock FPGA: stands in for the Nexys 3 on the network, using ref/brain.py.

It speaks exactly the packets in docs/packets.md: listens on UDP 5006 for
5A / C0 / C1 / C2, keeps the camera grid band by band as the C1 packets
arrive, runs one brain step on each C2, and broadcasts a B5 reply on UDP
5005. It also repeats the last reply about twice a second, like the board.

Replies are broadcast on the board's subnet (192.168.10.255), which macOS
also delivers to every local listener, so the bridge and the viewer both
get them without anything going out on Wi-Fi. If that subnet is down (no
cable), it falls back to 127.0.0.1, which only one local listener receives.

  python3 bridge/mock_fpga.py                 # replies to 192.168.10.255
  python3 bridge/mock_fpga.py --drop 0.05     # lose 5% of incoming packets
"""

import argparse
import os
import random
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ref'))
import brain as ref_brain  # noqa: E402
import packets as P  # noqa: E402

HEARTBEAT_S = 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--reply-to', default=P.PC_BROADCAST)
    parser.add_argument('--drop', type=float, default=0.0,
                        help='fraction of incoming packets to drop (like a bad CRC)')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(('', P.CMD_PORT))
    rx.settimeout(HEARTBEAT_S)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    brain = ref_brain.Brain()
    camera_grid = 0
    last_reply = None
    last_sent = 0.0
    print(f'mock FPGA: listening on UDP {P.CMD_PORT}, replying to '
          f'{args.reply_to}:{P.STATE_PORT} (Ctrl-C to stop)')

    reply_to = args.reply_to

    def send(reply):
        nonlocal last_sent, reply_to
        try:
            tx.sendto(reply, (reply_to, P.STATE_PORT))
        except OSError as exc:
            print(f'cannot send to {reply_to} ({exc.strerror}); using 127.0.0.1 '
                  '(only one local listener will get replies)')
            reply_to = '127.0.0.1'
            tx.sendto(reply, (reply_to, P.STATE_PORT))
        last_sent = time.monotonic()

    while True:
        try:
            data, _ = rx.recvfrom(2048)
        except socket.timeout:
            data = None
        if data is not None and random.random() >= args.drop:
            cmd = P.parse_command(data)
            if cmd is None:
                pass
            elif cmd[0] == 'edit':
                brain.edit(cmd[1], cmd[2])
            elif cmd[0] == 'reset':
                brain.reset()
                camera_grid = 0
                print('reset')
            elif cmd[0] == 'camera':
                _, _, band, pixels = cmd
                row = ref_brain.detect_band(pixels)
                camera_grid = (camera_grid & ~(0xFF << band * 8)) | (row << band * 8)
            elif cmd[0] == 'pose':
                _, frame, x, y, h = cmd
                wheels = brain.step_grid(camera_grid, x, y, h)
                last_reply = P.reply_from_brain(frame, brain, wheels)
                send(last_reply)
                if not args.quiet:
                    for event in brain.events:
                        if event[0] == 'plan':
                            print(f'frame {frame:3d}: plan from {event[1]}: '
                                  f'{event[3] if event[2] else "NO ROUTE"}')
        if last_reply and time.monotonic() - last_sent >= HEARTBEAT_S:
            send(last_reply)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
