#!/usr/bin/env python3
"""Live view of the flow-field brain: receives its B6 reply broadcasts over UDP.

  python3 tools/ff_viewer.py                  # board or bridge/mock_fpga_ff.py
  python3 tools/ff_viewer.py --text           # print replies instead of drawing
  python3 tools/ff_viewer.py --target 127.0.0.1   # send wall clicks to the mock on this Mac

Draws the 16x16 warehouse: racks (known to the FPGA), obstacles it has seen,
the cells it thinks something moved through (red), the robot, the waypoint it
is driving to, and the flow arrows. The arrows are drawn here from the masks in
the reply with the same code the FPGA is tested against (ref/flowfield.py);
the decisions themselves are made on the FPGA. Click a cell to add or remove
a wall there.
"""
import argparse
import os
import socket
import sys
import time
import tkinter as tk

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ref'))
from flowfield import cost_field, flow_step, weights  # noqa: E402
from packets_ff import pack_edit, parse_reply  # noqa: E402
from warehouse import CELLS, GOAL, GRID, INF, RACK_MASK  # noqa: E402

PX = 40
COLORS = dict(floor='#f4f1ea', rack='#8a8fa3', blocked='#e0a458', mover='#e8756d',
              halo='#f6d7d3', goal='#f2cc3c', robot='#2b6cb0', target='#2b6cb0', arrow='#6b7280')


def open_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, 'SO_REUSEPORT'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('', port))
    return sock


def describe(r):
    return (f'robot={r["robot_cell"]} target={r["target_cell"]} {r["status"]} '
            f'wheels={r["left"]:+d}/{r["right"]:+d} decisions={r["replans"]} '
            f'blocked={bin(r["blocked"]).count("1")} movers={bin(r["movers"]).count("1")}')


def flow_for(reply):
    keep_free = (1 << reply['robot_cell']) | (1 << GOAL)
    blocked = (reply['blocked'] | RACK_MASK) & ~keep_free
    field, _ = cost_field(weights(blocked, reply['movers']))
    return field


class View:
    def __init__(self, root, sock, target):
        self.sock, self.target = sock, target
        self.canvas = tk.Canvas(root, width=GRID * PX, height=GRID * PX, bg=COLORS['floor'])
        self.canvas.pack()
        self.label = tk.Label(root, text='Waiting for B6 replies on UDP port 5005. '
                                         'Click a cell to add/remove a wall.', anchor='w')
        self.label.pack(fill='x')
        self.reply, self.field, self.key = None, None, None
        self.last = None
        self.canvas.bind('<Button-1>', self.click)
        sock.setblocking(False)
        self.poll()

    def click(self, event):
        cell = event.y // PX * GRID + event.x // PX
        if not 0 <= cell < CELLS or (RACK_MASK >> cell) & 1 or cell == GOAL or not self.reply:
            return
        add = not (self.reply['blocked'] >> cell) & 1
        try:
            self.sock.sendto(pack_edit(add, cell), (self.target, 5006))
            self.label.config(text=f'Sent: {"add" if add else "remove"} wall at cell {cell}')
        except OSError as exc:
            self.label.config(text=f'Could not send ({exc.strerror}). Is the Ethernet link up?')

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            r = parse_reply(data)
            if r:
                self.reply, self.last = r, time.monotonic()
                key = (r['blocked'], r['movers'], r['robot_cell'])
                if key != self.key:
                    self.key, self.field = key, flow_for(r)
                self.label.config(text=describe(r))
                self.draw()
        if self.last and time.monotonic() - self.last > 2:
            self.label.config(text='No data from the FPGA for a while: check the cable and power')
        self.canvas.after(30, self.poll)

    def draw(self):
        r, c = self.reply, self.canvas
        c.delete('all')
        halo = 0
        for _ in range(3):   # same reach as the planner's halo
            m = r['movers'] if not halo else halo
            halo = (m | (m << 1) | (m >> 1) | (m << GRID) | (m >> GRID)) & ((1 << CELLS) - 1)
        for cell in range(CELLS):
            row, col = divmod(cell, GRID)
            x0, y0 = col * PX, row * PX
            color = None
            if (RACK_MASK >> cell) & 1:
                color = COLORS['rack']
            elif (r['blocked'] >> cell) & 1:
                color = COLORS['blocked']
            elif (r['movers'] >> cell) & 1:
                color = COLORS['mover']
            elif (halo >> cell) & 1:
                color = COLORS['halo']
            elif cell == GOAL:
                color = COLORS['goal']
            c.create_rectangle(x0, y0, x0 + PX, y0 + PX, outline='#d9d4c7',
                               fill=color or COLORS['floor'])
            if self.field and color not in (COLORS['rack'], COLORS['blocked']) and cell != GOAL:
                nxt = flow_step(self.field, cell)
                if nxt is not None and self.field[cell] < INF:
                    nr, nc = divmod(nxt, GRID)
                    c.create_line(x0 + PX / 2 - (nc - col) * 6, y0 + PX / 2 - (nr - row) * 6,
                                  x0 + PX / 2 + (nc - col) * 9, y0 + PX / 2 + (nr - row) * 9,
                                  fill=COLORS['arrow'], arrow='last', width=1)
        if r['status'] == 'running':
            tr, tc = divmod(r['target_cell'], GRID)
            c.create_rectangle(tc * PX + 3, tr * PX + 3, tc * PX + PX - 3, tr * PX + PX - 3,
                               outline=COLORS['target'], width=2)
        rr, rc = divmod(r['robot_cell'], GRID)
        c.create_oval(rc * PX + 9, rr * PX + 9, rc * PX + PX - 9, rr * PX + PX - 9,
                      fill=COLORS['robot'], outline='')


def print_replies(sock):
    last = None
    print(f'listening on UDP port {sock.getsockname()[1]} (Ctrl-C to stop)')
    while True:
        data, (ip, _) = sock.recvfrom(2048)
        r = parse_reply(data)
        if r:
            key = {k: v for k, v in r.items() if k != 'frame'}
            if key != last:
                last = key
                print(f'{time.strftime("%X")} {ip}: frame={r["frame"]} {describe(r)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('--text', action='store_true')
    parser.add_argument('--target', default='192.168.10.255', help='where wall clicks are sent')
    args = parser.parse_args()
    sock = open_socket(args.port)
    if args.text:
        try:
            print_replies(sock)
        except KeyboardInterrupt:
            pass
        return
    root = tk.Tk()
    root.title('FPGA flow-field warehouse robot')
    View(root, sock, args.target)
    root.mainloop()


if __name__ == '__main__':
    main()
