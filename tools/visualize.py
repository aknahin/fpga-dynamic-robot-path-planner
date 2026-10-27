#!/usr/bin/env python3
"""View simulation events, or live path masks from Nexys 3 UART."""
import argparse
import csv
import tkinter as tk

MAPS = [0, 0x0000fe007f000000, 0x0000fe00ff000000, 0x0000fe007f000000]
COLORS = {'empty': '#172337', 'wall': '#627287', 'explore': '#6389ad',
          'path': '#51df9b', 'start': '#3688f5', 'goal': '#f9cc4c',
          'trail': '#2c4a5e', 'mover': '#f08a3c'}


def read_frames(filename):
    """FRAME rows written by tb_random_dynamic.v."""
    frames = []
    with open(filename, newline='') as handle:
        for row in csv.reader(handle):
            if row and row[0] == 'FRAME':
                step, robot, mover = map(int, row[1:4])
                walls, path, explored = (int(v, 16) for v in row[4:7])
                frames.append((step, robot, mover, walls, path, explored, row[7]))
    return frames


def print_frames(frames):
    for step, robot, mover, walls, path, explored, note in frames:
        print(f'step {step}: {note}')
        for r in range(8):
            line = ''
            for c in range(8):
                n = r*8 + c
                ch = '#' if walls >> n & 1 else '*' if path >> n & 1 else '.'
                if n == 63: ch = 'G'
                if n == mover: ch = 'M'
                if n == robot: ch = 'R'
                line += ch + ' '
            print('  ' + line)
        print()


GIF_PALETTE = ['#0e1624', '#172337', '#627287', '#51df9b', '#2c4a5e', '#f35c69',
               '#f08a3c', '#f9cc4c', '#3d5a7a', '#b26bff', '#ffffff', '#3688f5',
               '#51df9b', '#f9cc4c', '#ff3b3b', '#000000']
BAR = {'move': 12, 'planned': 11, 'start': 11, 'goal reached': 13,
       'route blocked': 14, 'retry': 14, 'waiting': 13, 'no route - waiting': 14}


def lzw_encode(pixels, min_size=4):
    clear, end = 1 << min_size, (1 << min_size) + 1
    out, acc, nbits = bytearray(), 0, 0
    def emit(code, width):
        nonlocal acc, nbits
        acc |= code << nbits; nbits += width
        while nbits >= 8:
            out.append(acc & 255); acc >>= 8; nbits -= 8
    def reset():
        return {bytes([i]): i for i in range(clear)}, end + 1, min_size + 1
    table, nxt, width = reset()
    emit(clear, width)
    word = b''
    for p in pixels:
        grown = word + bytes([p])
        if grown in table:
            word = grown
            continue
        emit(table[word], width)
        if nxt == 4096:
            emit(clear, width)
            table, nxt, width = reset()
        else:
            table[grown] = nxt; nxt += 1
            if nxt > (1 << width) and width < 12: width += 1
        word = bytes([p])
    emit(table[word], width)
    emit(end, width)
    if nbits: out.append(acc & 255)
    return bytes(out)


def render_frame(frame, trail, cell=44):
    """One replay frame as palette indices; a status bar shows the frame type."""
    _, robot, mover, walls, path, explored, note = frame
    size, bar = 8 * cell, 14
    img = bytearray([0]) * (size * (size + bar))
    path_color = 9 if note in ('route blocked', 'retry') else 3
    for n in range(64):
        color = 1
        if trail >> n & 1: color = 4
        if explored >> n & 1: color = 8
        if path >> n & 1: color = path_color
        if walls >> n & 1: color = 2
        if n == 63: color = 7
        if n == mover: color = 6
        r, c = divmod(n, 8)
        for y in range(r*cell + 2, r*cell + cell - 2):
            img[y*size + c*cell + 2: y*size + c*cell + cell - 2] = bytes([color]) * (cell - 4)
        if n == robot:
            cy, cx, rad = r*cell + cell/2, c*cell + cell/2, cell * 0.3
            for y in range(r*cell, (r+1)*cell):
                for x in range(c*cell, (c+1)*cell):
                    d = ((y + .5 - cy)**2 + (x + .5 - cx)**2) ** .5
                    if d <= rad: img[y*size + x] = 5
                    elif d <= rad + 2: img[y*size + x] = 10
    img[size*size:] = bytes([BAR.get(note, 11)]) * (size * bar)
    return size, size + bar, bytes(img)


def write_gif(frames, filename):
    trail, images = 0, []
    for frame in frames:
        trail |= 1 << frame[1]
        images.append((render_frame(frame, trail), frame[6]))
    width, height = images[0][0][:2]
    data = bytearray(b'GIF89a' + width.to_bytes(2, 'little') + height.to_bytes(2, 'little')
                     + bytes([0xF3, 0, 0]))
    for color in GIF_PALETTE:
        data += bytes.fromhex(color[1:])
    data += b'\x21\xFF\x0BNETSCAPE2.0\x03\x01\x00\x00\x00'  # loop forever
    for i, ((_, _, pixels), note) in enumerate(images):
        delay = 250 if i == len(images) - 1 else 100 if note in ('route blocked', 'retry', 'waiting') else 60 if note == 'planned' else 35
        data += b'\x21\xF9\x04\x00' + delay.to_bytes(2, 'little') + b'\x00\x00'
        data += b'\x2C\x00\x00\x00\x00' + width.to_bytes(2, 'little') + height.to_bytes(2, 'little') + b'\x00'
        data += b'\x04'
        encoded = lzw_encode(pixels)
        for k in range(0, len(encoded), 255):
            chunk = encoded[k:k+255]
            data += bytes([len(chunk)]) + chunk
        data += b'\x00'
    data += b'\x3B'
    with open(filename, 'wb') as handle:
        handle.write(data)
    print(f'wrote {filename}: {len(images)} frames')


class Viewer:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=480, height=480, bg='#172337')
        self.canvas.pack()
        self.label = tk.Label(root, text='Waiting for data', font=('Arial', 15))
        self.label.pack()
        self.obstacles = self.explored = self.path = self.trail = 0
        self.robot = 0
        self.mover = None
        self.draw()

    def draw(self):
        self.canvas.delete('all')
        for n in range(64):
            color = COLORS['empty']
            if self.trail >> n & 1: color = COLORS['trail']
            if self.obstacles >> n & 1: color = COLORS['wall']
            if self.explored >> n & 1: color = COLORS['explore']
            if self.path >> n & 1: color = COLORS['path']
            if n == 0: color = COLORS['start']
            if n == 63: color = COLORS['goal']
            if n == self.mover: color = COLORS['mover']
            r, c = divmod(n, 8)
            self.canvas.create_rectangle(c*60+2, r*60+2, c*60+58, r*60+58,
                                         fill=color, outline='')
            if n == self.robot:
                self.canvas.create_oval(c*60+14, r*60+14, c*60+46, r*60+46,
                                        fill='#f35c69', outline='white', width=2)

    def simulation(self, filename):
        with open(filename, newline='') as handle:
            entries = list(csv.reader(handle))
        def advance(index=0):
            if index >= len(entries): return
            kind, scenario, *values = entries[index]
            if kind == 'MAP':
                self.obstacles = int(values[0], 16)
                self.explored = self.path = 0
                self.label.config(text=f'Simulation case {scenario}: planning')
            elif kind == 'EVENT':
                typ, cell = map(int, values)
                if typ == 0: self.explored |= 1 << cell
                if typ == 1: self.path |= 1 << cell
            elif kind == 'RESULT':
                found, length, expanded = map(int, values)
                self.label.config(text=f'Case {scenario}: found={found}, length={length}, explored={expanded}')
            self.draw()
            self.root.after(25 if kind == 'EVENT' else 700, advance, index+1)
        advance()

    def replay(self, frames):
        def advance(index=0):
            if index >= len(frames): return
            step, self.robot, self.mover, self.obstacles, self.path, self.explored, note = frames[index]
            self.trail |= 1 << self.robot
            self.label.config(text=f'Step {step}: {note}')
            self.draw()
            delay = 900 if note in ('route blocked', 'planned', 'waiting') else 450
            self.root.after(delay, advance, index+1)
        advance()

    def live(self, port):
        try:
            import serial
        except ImportError as exc:
            raise SystemExit('Install pyserial: python3 -m pip install pyserial') from exc
        link = serial.Serial(port, 115200, timeout=0)
        buffer = bytearray()
        def poll():
            buffer.extend(link.read(link.in_waiting or 1))
            while buffer:
                if buffer[0] != 0xA5:
                    del buffer[0]
                    continue
                if len(buffer) < 21: break
                packet = bytes(buffer[:21]); del buffer[:21]
                robot, found, length, explored = packet[1:5]
                if robot > 63 or found > 1: continue
                self.robot = robot
                self.obstacles = int.from_bytes(packet[5:13], 'little')
                self.path = int.from_bytes(packet[13:21], 'little')
                self.explored = 0
                self.label.config(text=f'FPGA robot={robot}, found={found}, route={length}, explored={explored}')
                self.draw()
            self.root.after(30, poll)
        poll()


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--sim', metavar='EVENTS.CSV')
    group.add_argument('--port', help='UART device, e.g. /dev/cu.usbserial-XXXX')
    group.add_argument('--replay', metavar='RANDOM_DYNAMIC.CSV',
                       help='robot run from tb_random_dynamic')
    parser.add_argument('--text', action='store_true', help='print --replay frames instead')
    parser.add_argument('--gif', metavar='OUT.GIF', help='save --replay frames as an animated GIF')
    args = parser.parse_args()
    if args.replay and args.text:
        return print_frames(read_frames(args.replay))
    if args.replay and args.gif:
        return write_gif(read_frames(args.replay), args.gif)
    root = tk.Tk(); root.title('FPGA dynamic path planner')
    view = Viewer(root)
    if args.sim: view.simulation(args.sim)
    elif args.replay: view.replay(read_frames(args.replay))
    else: view.live(args.port)
    root.mainloop()


if __name__ == '__main__': main()
