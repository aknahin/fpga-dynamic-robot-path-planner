"""UDP payload formats between the Mac and the FPGA for the flow-field brain.

Executable form of docs/packets_ff.md. The bridge, the mock FPGA and the
testbenches all build and parse packets through this module.

PC -> FPGA, broadcast to 192.168.10.255, UDP port 5006
  5A  wall edit     5A, op (1 add / 0 remove), cell (0..255)               3 bytes
  C0  reset brain   C0                                                     1 byte
  C3  camera strip  C3, frame, cam (0 front, 1 left, 2 back, 3 right),
                    64 depth bytes (2 rows x 32 columns, row by row)      67 bytes
  C2  pose / go     C2, frame, x_mm i16, y_mm i16, heading u16             8 bytes

FPGA -> PC, broadcast from 192.168.10.2, UDP port 5005
  B6  brain reply   B6, frame, left i8, right i8, status, robot cell,
                    target cell, replans, blocked u256, movers u256       72 bytes

Multi-byte fields are little endian. Cell = row*16 + col, row 0 north.
The racks are known to the FPGA, so they are not in the reply masks.
"""

import struct

import numpy as np

from warehouse import CAM_BYTES, CAM_COLS, CAM_ROWS, CAMERAS

CMD_PORT = 5006
STATE_PORT = 5005
PC_BROADCAST = '192.168.10.255'

EDIT, RESET, CAMERA, POSE = 0x5A, 0xC0, 0xC3, 0xC2
REPLY = 0xB6
REPLY_BYTES = 72

STATUS_CODE = {'wait_map': 0, 'running': 1, 'no_route': 2, 'at_goal': 3}
STATUS_NAME = {v: k for k, v in STATUS_CODE.items()}

_POSE = struct.Struct('<BBhhH')
_HEAD = struct.Struct('<BBbbBBBB')


def pack_edit(add, cell):
    return bytes([EDIT, 1 if add else 0, cell])


def pack_reset():
    return bytes([RESET])


def pack_camera(frame, cam, image):
    """image: CAM_ROWS x CAM_COLS uint8 -> one C3 payload."""
    image = np.asarray(image, dtype=np.uint8).reshape(CAM_ROWS, CAM_COLS)
    return bytes([CAMERA, frame & 0xFF, cam]) + image.tobytes()


def pack_cameras(frame, images):
    return [pack_camera(frame, cam, images[cam]) for cam in range(CAMERAS)]


def pack_pose(frame, x_mm, y_mm, heading):
    return _POSE.pack(POSE, frame & 0xFF, x_mm, y_mm, heading)


def parse_command(data):
    """FPGA-side parse. Returns a tuple, or None if the FPGA must ignore it."""
    if len(data) >= 3 and data[0] == EDIT and data[1] < 2:
        return ('edit', bool(data[1]), data[2])
    if len(data) >= 1 and data[0] == RESET:
        return ('reset',)
    if len(data) >= 3 + CAM_BYTES and data[0] == CAMERA and data[2] < CAMERAS:
        image = np.frombuffer(data[3:3 + CAM_BYTES], dtype=np.uint8).reshape(CAM_ROWS, CAM_COLS)
        return ('camera', data[1], data[2], image)
    if len(data) >= _POSE.size and data[0] == POSE:
        _, frame, x, y, h = _POSE.unpack_from(data)
        return ('pose', frame, x, y, h)
    return None


def pack_reply(frame, left, right, status, robot_cell, target_cell, replans, blocked, movers):
    return (_HEAD.pack(REPLY, frame & 0xFF, left, right, STATUS_CODE[status],
                       robot_cell, target_cell, replans & 0xFF)
            + blocked.to_bytes(32, 'little') + movers.to_bytes(32, 'little'))


def parse_reply(data):
    """PC-side parse of a B6 reply. Returns a dict, or None."""
    if len(data) < REPLY_BYTES or data[0] != REPLY:
        return None
    _, frame, left, right, status, robot, target, replans = _HEAD.unpack_from(data)
    if status not in STATUS_NAME:
        return None
    return dict(frame=frame, left=left, right=right, status=STATUS_NAME[status],
                robot_cell=robot, target_cell=target, replans=replans,
                blocked=int.from_bytes(data[8:40], 'little'),
                movers=int.from_bytes(data[40:72], 'little'))


def reply_from_brain(frame, brain, wheels):
    """B6 payload for one ref/brain_ff.py step (used by the mock FPGA)."""
    return pack_reply(frame, wheels[0], wheels[1], brain.status, brain.robot_cell,
                      brain.target_cell(), brain.replans, brain.blocked_mask(),
                      brain.mover_mask())
