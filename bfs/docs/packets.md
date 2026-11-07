# Packet formats (Phase 2)

These are the UDP payloads between the Mac and the Nexys 3 when the FPGA drives the simulated robot. The Mac only sends sensor data and applies the wheel speeds it gets back; every decision is made on the FPGA. `ref/packets.py` builds and parses all of them, and the bridge, the mock FPGA and the testbenches use it, so this page and that file must stay in step.

## Link

Same cable and addresses as Phase 1:

| | Mac | FPGA |
|---|---|---|
| IP | 192.168.10.1 | 192.168.10.2 |
| Sends to | 192.168.10.255 (broadcast), UDP port 5006 | 255.255.255.255 (broadcast), UDP port 5005 |
| MAC | the adapter's own | 02:00:00:00:00:01 |

Both sides broadcast, so neither needs ARP. The FPGA checks the Ethernet CRC of every frame it receives, checks that it is IPv4/UDP to port 5006, and then looks at the first payload byte to tell the packets apart. Anything else is dropped. All multi-byte fields are little endian.

## Mac to FPGA (UDP port 5006)

| First byte | Packet | Length | Fields |
|---|---|---|---|
| `5A` | Wall edit | 3 | `5A`, op (1 add, 0 remove), cell (0..63) |
| `C0` | Reset brain | 1 | `C0` |
| `C1` | Camera band | 515 | `C1`, frame, band (0..7), 512 depth bytes |
| `C2` | Pose, end of frame | 8 | `C2`, frame, x_mm (int16), y_mm (int16), heading (uint16) |

**`5A` wall edit** is the Phase 1 packet, unchanged. A clicked wall is added to whatever the camera sees. Clicks on the start (0), the goal (63) or the robot's current cell are ignored, as in Phase 1. Clicked walls stay after a `C0` reset.

**`C0` reset** clears the route and puts the brain back into "waiting for the first frame". The bridge sends it when a Webots run starts.

**`C1` camera band** carries one eighth of the 64x64 depth image: image rows `band*8` to `band*8+7`, row by row, each row west to east. Band 0 is the north edge. One band covers exactly one row of grid cells, so the FPGA can finish that row of cells inside the packet:

- A depth byte is depth / 2 cm, rounded and clamped to 255. The floor reads 150 (3.00 m), wall tops read 137 or 138, and the e-puck reads about 147.
- A pixel is "high" when its byte is below 143.
- While the packet streams in, the FPGA keeps one counter per cell column (8 counters, pixel column / 8). At the end of the packet, a cell is blocked when its counter is 32 or more (half of its 64 pixels).
- The 8 new bits replace that row of the camera grid only if the frame's CRC is good. A lost or bad band leaves that row as it was in the previous frame.

**`C2` pose** closes the frame. It carries the robot pose:

- x_mm, y_mm: position in millimetres, arena centre at (0, 0), +x east, +y north. The arena spans -600 to +600 on both axes.
- heading: 16-bit angle counter-clockwise from east: 0 east, 16384 north, 32768 west, 49152 south.

When the FPGA gets a `C2`, it runs one brain step with the current grid and this pose, then sends a `B5` reply with the same frame number.

Each simulation step (32 ms) the bridge sends the 8 bands and then the pose: about 4.6 KB per frame, about 1.2 Mbit/s at 31 frames per second. That fits easily even if the PHY links at 10 Mbit/s.

## FPGA to Mac (UDP port 5005)

| First byte | Packet | Length | Sent by |
|---|---|---|---|
| `A5` | Phase 1 state | 21 | Phase 1 bitstream only (`nexys3_top_lan`), unchanged |
| `B5` | Brain reply | 24 | Phase 2 bitstream, after each `C2` and every ~0.5 s |

**`B5` brain reply**:

| Byte | Field | Meaning |
|---|---|---|
| 0 | `B5` | |
| 1 | frame | copied from the `C2` it answers |
| 2 | left | left wheel speed, int8, 0.05 rad/s per step |
| 3 | right | right wheel speed, int8, 0.05 rad/s per step |
| 4 | status | 0 waiting for a frame, 1 running, 2 no route, 3 at goal (the same codes as LD1 LD0 on the board) |
| 5 | robot cell | cell under the robot, from the pose |
| 6 | target cell | the waypoint being driven to (the robot cell when not running) |
| 7 | replans | number of BFS runs since reset, wraps at 256 |
| 8..15 | obstacles | 64-bit grid used for planning (camera + clicked walls; robot cell and goal cleared) |
| 16..23 | route | 64-bit mask of the route still to drive, from the cell just left to the goal |

The bridge applies `left` and `right` to the Webots motors unchanged. If no `B5` for the current frame arrives within 200 ms, it stops the wheels and goes on with the next frame. The FPGA also repeats its last `B5` about twice a second, so the viewer can be started at any time.

## What the FPGA does on each `C2`

This is `ref/brain.py`, which the Verilog must match exactly:

1. Robot cell = column `(x_mm + 600) / 150`, row `(600 - y_mm) / 150`, each clamped to 0..7.
2. Obstacles = camera grid OR clicked walls, with the robot cell and the goal cleared.
3. Replan with BFS from the robot cell (the same `bfs_engine` as Phase 1) when:
   - it is the first frame after reset, or
   - a newly blocked cell lies on the route still to drive, or
   - there was no route and the grid changed.
4. Drive to the next waypoint (the centre of the next route cell):
   - When the robot is within 15 mm of it along the travel direction, move on to the next one. After the last one, stop: at goal.
   - Aim = travel direction minus a lane correction (sideways offset in mm × 64, clamped to ±4096).
   - If the heading error is above 2731 (about 15°), turn in place at ±40.
   - Otherwise drive at 100 ± (error >> 6).

The default Webots run, recorded in `sim/ref_run.npz`, is the test vector set: every frame's depth image, pose and the wheel speeds the reference produced. `python3 ref/check_run.py` replays it.

## Trying it without the board

`bridge/mock_fpga.py` stands in for the board on the network. It answers exactly these packets, using `ref/brain.py`.

```sh
python3 bridge/mock_fpga.py                                  # terminal 1: the mock FPGA
python3 tools/lan_viewer.py --target 127.0.0.1               # terminal 2: grid + route as the "FPGA" sees it
HIL_BRAIN=udp /Applications/Webots.app/Contents/MacOS/webots webots/worlds/hil_arena.wbt   # terminal 3
python3 ref/check_run.py sim/udp_run.npz                     # afterwards
```

`HIL_BRAIN=udp` switches the e-puck controller from the in-process reference to the network. `HIL_TARGET` sets where the packets go: `127.0.0.1` by default, `192.168.10.255` for the board. Without `HIL_BRAIN`, the world runs the reference in-process, as before.

`python3 bridge/replay_frames.py [run.npz] [--target IP]` sends a recorded run over UDP without Webots and checks every reply field against `ref/brain.py`. Against the board, this is the bit-exact test for the Verilog brain. `mock_fpga.py --drop 0.05` drops 5% of incoming packets to test how the bridge copes with loss.

## The Verilog side

| File | What it does |
|---|---|
| `rtl/eth_hil_rx.v` | Receives `5A`, `C0`, `C1`, `C2` over MII. Counts the high pixels of each `C1` band per cell column as the bytes stream in, so no image is ever stored. |
| `rtl/hil_brain.v` | `ref/brain.py` in hardware, around the Phase 1 `bfs_engine`: robot cell from the pose, replan decision, waypoints, wheel speeds, the `B5` payload. |
| `rtl/eth_udp_tx.v` | The Phase 1 transmitter with a `PAYLOAD_BYTES` parameter (21 for Phase 1, 24 for `B5`). |
| `rtl/nexys3_top_hil.v` | Phase 2 top level. Sends each `B5` reply and repeats the last one every 0.5 s. |
| `constraints/nexys3_hil.ucf` | The Ethernet pins from `nexys3_lan.ucf`, without the button and switches. |

LEDs: LD1 LD0 show the status (00 waiting for a frame, 01 running, 10 no route, 11 at goal). LD2 is on while BFS plans, LD3 flips per frame sent, LD4 is on once the PHY is out of reset, LD5 flips per packet received, and LD6 blinks while the 100 MHz clock runs.

Simulation, checking every reply byte against the reference (about 3 minutes for a full run):

```sh
python3 ref/make_vectors.py sim/udp_run.npz          # recording -> sim/hil_vectors.txt
iverilog -g2012 -Wall -s tb_nexys3_hil -o sim/hil.out rtl/neighbor_check.v rtl/bfs_engine.v rtl/eth_udp_tx.v rtl/eth_hil_rx.v rtl/hil_brain.v rtl/nexys3_top_hil.v tb/tb_nexys3_hil.v
vvp sim/hil.out                                      # +max_frames=50 for a quick run
```

`make_vectors.py` also mixes in a wrong-port band and a bad-CRC band (both must be ignored), a lost band, a wall click on the route and its removal, and a reset in the middle of the run.

Building it in ISE: same steps as the Phase 1 Ethernet version, with `nexys3_top_hil` as the top module, the files `rtl/neighbor_check.v`, `rtl/bfs_engine.v`, `rtl/eth_udp_tx.v`, `rtl/eth_hil_rx.v`, `rtl/hil_brain.v`, `rtl/nexys3_top_hil.v`, and `constraints/nexys3_hil.ucf`. Set "FPGA Start-Up Clock" to JTAG Clock and "Unused IOB Pins" to Float.
