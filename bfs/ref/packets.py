"""UDP payload formats between the Mac and the FPGA (Phase 2).

This module is the executable form of docs/packets.md. The bridge, the
mock FPGA and the testbenches all build and parse packets through it.

PC -> FPGA, broadcast to 192.168.10.255, UDP port 5006
  5A  wall edit      5A, op (1 add / 0 remove), cell                    3 bytes
  C0  reset brain    C0                                                 1 byte
  C1  camera band    C1, frame, band (0..7), 512 depth bytes          515 bytes
  C2  pose / go      C2, frame, x_mm i16, y_mm i16, heading u16         8 bytes

FPGA -> PC, broadcast from 192.168.10.2, UDP port 5005
  A5  Phase 1 state  (unchanged, Phase 1 bitstream only)               21 bytes
  B5  brain reply    B5, frame, left i8, right i8, status, robot cell,
                     target cell, replans, obstacles u64, route u64    24 bytes

Multi-byte fields are little endian.
"""

import struct

import numpy as np

CMD_PORT = 5006
STATE_PORT = 5005
PC_BROADCAST = '192.168.10.255'

EDIT, RESET, CAMERA, POSE = 0x5A, 0xC0, 0xC1, 0xC2
PHASE1_STATE, REPLY = 0xA5, 0xB5

BANDS = 8                 # one band = one row of grid cells = 8 image rows
BAND_BYTES = 8 * 64

# B5 status codes; the same 2-bit values as the board's LD1 LD0.
STATUS_CODE = {'wait_map': 0, 'running': 1, 'no_route': 2, 'at_goal': 3}
STATUS_NAME = {v: k for k, v in STATUS_CODE.items()}

_POSE = struct.Struct('<BBhhH')
_REPLY = struct.Struct('<BBbbBBBBQQ')


# ---- PC -> FPGA ----------------------------------------------------------

def pack_edit(add, cell):
    return bytes([EDIT, 1 if add else 0, cell])


def pack_reset():
    return bytes([RESET])


def pack_camera(frame, depth_u8):
    """64x64 uint8 depth -> list of 8 C1 payloads, band 0 (north) first."""
    depth_u8 = np.asarray(depth_u8, dtype=np.uint8)
    return [bytes([CAMERA, frame & 0xFF, band]) + depth_u8[band * 8:band * 8 + 8].tobytes()
            for band in range(BANDS)]


def pack_pose(frame, x_mm, y_mm, heading):
    return _POSE.pack(POSE, frame & 0xFF, x_mm, y_mm, heading)


def parse_command(data):
    """FPGA-side parse. Returns a tuple, or None if the FPGA must ignore it."""
    if len(data) >= 3 and data[0] == EDIT and data[1] < 2 and data[2] < 64:
        return ('edit', bool(data[1]), data[2])
    if len(data) >= 1 and data[0] == RESET:
        return ('reset',)
    if len(data) >= 3 + BAND_BYTES and data[0] == CAMERA and data[2] < BANDS:
        pixels = np.frombuffer(data[3:3 + BAND_BYTES], dtype=np.uint8).reshape(8, 64)
        return ('camera', data[1], data[2], pixels)
    if len(data) >= _POSE.size and data[0] == POSE:
        _, frame, x, y, h = _POSE.unpack_from(data)
        return ('pose', frame, x, y, h)
    return None


# ---- FPGA -> PC ----------------------------------------------------------

def pack_reply(frame, left, right, status, robot_cell, target_cell, replans,
               obstacles, route):
    return _REPLY.pack(REPLY, frame & 0xFF, left, right, STATUS_CODE[status],
                       robot_cell, target_cell, replans & 0xFF, obstacles, route)


def parse_reply(data):
    """PC-side parse of a B5 reply. Returns a dict, or None."""
    if len(data) < _REPLY.size or data[0] != REPLY:
        return None
    _, frame, left, right, status, robot, target, replans, obstacles, route = \
        _REPLY.unpack_from(data)
    if status not in STATUS_NAME or robot > 63 or target > 63:
        return None
    return dict(frame=frame, left=left, right=right, status=STATUS_NAME[status],
                robot_cell=robot, target_cell=target, replans=replans,
                obstacles=obstacles, route=route)


def reply_from_brain(frame, brain, wheels):
    """B5 payload for one ref/brain.py step (used by the mock FPGA)."""
    return pack_reply(frame, wheels[0], wheels[1], brain.status, brain.robot_cell,
                      brain.target_cell(), brain.replans, brain.obstacles,
                      brain.route_mask())
