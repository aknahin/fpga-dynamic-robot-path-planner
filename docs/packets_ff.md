# Packet formats (flow-field warehouse robot)

These are the UDP payloads between the Mac and the Nexys 3 when the FPGA drives the simulated warehouse robot. The Mac only sends sensor data and applies the wheel speeds it gets back; every decision is made on the FPGA. `ref/packets_ff.py` builds and parses all of them, and the bridge, the mock FPGA and the testbenches use it, so this page and that file must stay in step. The older 8x8 / BFS formats are in `bfs/docs/packets.md`.

## Link

Same cable and addresses as before:

| | Mac | FPGA |
|---|---|---|
| IP | 192.168.10.1 | 192.168.10.2 |
| Sends to | 192.168.10.255 (broadcast), UDP port 5006 | 255.255.255.255 (broadcast), UDP port 5005 |
| MAC | the adapter's own | 02:00:00:00:00:01 |

The FPGA checks the Ethernet CRC of every frame, checks that it is IPv4/UDP to port 5006, and then looks at the first payload byte. Anything else is dropped. Multi-byte fields are little endian. Cell = row * 16 + col, row 0 is the north edge (+y), column 0 the west edge (-x); the delivery zone is cell 255 (south-east corner).

## Mac to FPGA (UDP port 5006)

| First byte | Packet | Length | Fields |
|---|---|---|---|
| `5A` | Wall edit | 3 | `5A`, op (1 add, 0 remove), cell (0..255) |
| `C0` | Reset brain | 1 | `C0` |
| `C3` | Camera strip | 67 | `C3`, frame, cam (0 front, 1 left, 2 back, 3 right), 64 depth bytes |
| `C2` | Pose, end of frame | 8 | `C2`, frame, x_mm (int16), y_mm (int16), heading (uint16) |

**`C3` camera strip** is one robot-mounted depth camera: 2 rows x 32 columns, row by row, column 0 on the left of the camera's axis. Depth byte = planar depth / 6 mm, clamped to 255; 250 or more means nothing in range (1.5 m). The camera is horizontal, 7 cm above the floor, with a 90 degree field of view. The four cameras look along the robot's heading, 90 degrees left of it, behind it, and 90 degrees right of it. While the 64 bytes stream in, the FPGA keeps the smaller of the two rows per column; nothing else is stored. A lost strip leaves the previous one in place.

**`C2` pose** closes the frame and starts one brain step. x_mm, y_mm: millimetres, arena centre (0, 0), +x east, +y north, spanning -1200 to +1200. heading: 16-bit angle counter-clockwise from east (0 east, 16384 north, 32768 west, 49152 south). The Mac sends 4 strips and then the pose each simulation step (32 ms), about 0.3 KB per frame.

**`5A` wall edit** puts a wall on a cell by hand (it is added to what the cameras see). Clicks on a rack, on the goal or on the robot's current cell are ignored. Walls stay after a `C0` reset.

**`C0` reset** forgets everything the cameras showed and waits for the first frame. (The racks and the cart lanes are built into the FPGA.) The bridge sends it when a Webots run starts.

## FPGA to Mac (UDP port 5005)

**`B6` brain reply** (72 bytes), sent after each `C2` and every ~0.5 s:

| Byte | Field | Meaning |
|---|---|---|
| 0 | `B6` | |
| 1 | frame | copied from the `C2` it answers |
| 2 | left | left wheel speed, int8, 0.05 rad/s per step |
| 3 | right | right wheel speed, int8, 0.05 rad/s per step |
| 4 | status | 0 waiting for first frame, 1 running, 2 no route, 3 at goal (= LD1 LD0) |
| 5 | robot cell | cell under the robot |
| 6 | target cell | waypoint being driven to (the robot cell when not running) |
| 7 | decisions | how many times a waypoint was chosen from scratch, mod 256 |
| 8..39 | blocked | 256-bit mask, bit n = cell n: obstacles the cameras found, plus clicked walls (racks are not included; the viewer knows them) |
| 40..71 | movers | 256-bit mask: cells that something recently moved out of (flagged for 2 s) |

## What the FPGA does with them (ref/ is the exact model)

1. **Perception** (`ref/perception.py`, `rtl/ff_brain.v`): each of the 128 camera columns is turned into a ray. The ray is sampled in 16 steps. Cells it passes through the middle of (at least 20 mm from the cell edges) count as seen free; the cell it ends in counts as seen occupied (ignored if it is a rack or sits within 14 mm of a cell edge, which is where racks and the perimeter wall are). Per cell: 2-bit occupancy (+1 on a hit, -1 on a pass, blocked at 2 or more) and a 6-bit mover timer, set to 63 frames when a blocked cell empties. A cell that stays blocked is a box; one that empties is a mover. Unobserved cells fade one step per 32 frames only on the known cart lanes, or anywhere while there is no route (deadlock breaker); boxes elsewhere are remembered.
2. **Cost field** (`ref/flowfield.py`, `rtl/ff_brain.v`): a cost to the goal for every cell. Step cost 1, plus 10, 6 or 3 within 1, 2 or 3 cells of a mover timer; blocked cells are infinite. Built from scratch every frame by forward/backward sweeps until nothing changes (a reverse Dijkstra), at most about 37,000 clock cycles (0.37 ms).
3. **Flow**: the robot drives to the neighbouring cell with the lowest cost and picks the next one on arrival. It does not step straight back to the cell it just left unless that is cheaper by 8, which stops it dithering between two routes whose costs swap as obstacles move.
4. **Drive**: the same waypoint controller as the BFS version (`bfs/ref/brain.py`).
