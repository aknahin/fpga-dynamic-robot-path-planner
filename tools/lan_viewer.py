#!/usr/bin/env python3
"""Live view of the FPGA: receives its UDP broadcasts over Ethernet.

Phase 1 (nexys3_top_lan): the same 21-byte A5 packet as the UART version.
Phase 2: the 24-byte B5 brain reply (docs/packets.md), from the board or
from bridge/mock_fpga.py; it is drawn next to the Webots window.
Click a cell to add or remove a wall there; the FPGA replans if needed.
"""
import argparse
import os
import socket
import sys
import time
import tkinter as tk

from visualize import Viewer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ref'))
from packets import parse_reply  # noqa: E402


def parse_packet(packet):
    """Return (robot, found, length, explored, walls, path) or None."""
    if len(packet) != 21 or packet[0] != 0xA5:
        return None
    robot, found, length, explored = packet[1:5]
    if robot > 63 or found > 1:
        return None
    return (robot, found, length, explored,
            int.from_bytes(packet[5:13], 'little'), int.from_bytes(packet[13:21], 'little'))


def open_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, 'SO_REUSEPORT'):  # lets two viewers share the broadcasts
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('', port))
    return sock


def send_edit(sock, target, add, cell):
    """Ask the FPGA to add (or remove) a wall: UDP payload 5A, op, cell."""
    sock.sendto(bytes([0x5A, 1 if add else 0, cell]), (target, 5006))


def print_packets(sock):
    last = None
    print(f'listening on UDP port {sock.getsockname()[1]} (Ctrl-C to stop)')
    while True:
        data, (ip, _) = sock.recvfrom(2048)
        reply = parse_reply(data)
        if reply:
            key = {k: v for k, v in reply.items() if k != 'frame'}
            if key != last:
                last = key
                print(f'{time.strftime("%X")} {ip}: frame={reply["frame"]} {describe(reply)}')
            continue
        state = parse_packet(data)
        if state is None:
            print(f'{time.strftime("%X")} {ip}: ignored {len(data)} bytes')
            continue
        if state == last:
            continue  # heartbeat repeat
        last = state
        robot, found, length, explored, walls, path = state
        print(f'{time.strftime("%X")} {ip}: robot={robot} found={found} '
              f'route={length} explored={explored} walls={walls:016x} path={path:016x}')


def describe(reply):
    return (f'robot={reply["robot_cell"]} target={reply["target_cell"]} '
            f'{reply["status"]} wheels={reply["left"]:+d}/{reply["right"]:+d} '
            f'replans={reply["replans"]}')


def live(view, sock, target):
    sock.setblocking(False)
    def click(event):
        r, c = event.y // 60, event.x // 60
        if not (0 <= r < 8 and 0 <= c < 8):
            return
        cell = r*8 + c
        if cell in (0, 63, view.robot):
            view.label.config(text=f'Cell {cell} is the start, goal or robot')
            return
        add = not view.obstacles >> cell & 1
        try:
            send_edit(sock, target, add, cell)
        except OSError as exc:  # e.g. Ethernet link down
            view.label.config(text=f'Could not send ({exc.strerror}). Is the Ethernet link up?')
            return
        view.label.config(text=f'Sent: {"add" if add else "remove"} wall at cell {cell}')
    view.canvas.bind('<Button-1>', click)
    last_packet = [None]
    def poll():
        while True:
            try:
                data, (ip, _) = sock.recvfrom(2048)
            except BlockingIOError:
                break
            reply = parse_reply(data)
            if reply:
                last_packet[0] = time.monotonic()
                view.obstacles, view.path = reply['obstacles'], reply['route']
                view.robot, view.explored = reply['robot_cell'], 0
                view.label.config(text=f'FPGA {ip}: {describe(reply)}')
                view.draw()
                continue
            state = parse_packet(data)
            if state is None:
                continue
            last_packet[0] = time.monotonic()
            robot, found, length, explored, view.obstacles, view.path = state
            view.robot, view.explored = robot, 0
            view.label.config(text=f'FPGA {ip}: robot={robot}, found={found}, '
                                   f'route={length}, explored={explored}')
            view.draw()
        # The FPGA resends about twice a second, so a gap means no link.
        if last_packet[0] and time.monotonic() - last_packet[0] > 2:
            gap = int(time.monotonic() - last_packet[0])
            view.label.config(text=f'No data from the FPGA for {gap} s: check the Ethernet '
                                   'cable, the board power, and that UTM has not taken the adapter')
        view.root.after(30, poll)
    poll()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5005, help='UDP port (default 5005)')
    parser.add_argument('--text', action='store_true', help='print packets instead of drawing')
    parser.add_argument('--target', default='192.168.10.255',
                        help='where cell clicks are sent (default: LAN broadcast)')
    args = parser.parse_args()
    sock = open_socket(args.port)
    if args.text:
        try:
            print_packets(sock)
        except KeyboardInterrupt:
            pass
        return
    root = tk.Tk(); root.title('FPGA dynamic path planner (LAN)')
    view = Viewer(root)
    view.label.config(text=f'Waiting for UDP packets on port {args.port}. Click a cell to add/remove a wall.')
    live(view, sock, args.target)
    root.mainloop()


if __name__ == '__main__': main()
