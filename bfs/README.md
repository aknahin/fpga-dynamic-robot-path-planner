# BFS baseline (backup)

Snapshot of the Phase 2 design that plans with BFS on an 8x8 grid and an overhead depth camera.
Kept as a backup when the planner moved to a flow field. Run everything from inside this folder.

```sh
mkdir -p sim
iverilog -g2012 -Wall -s tb_bfs_engine -o sim/bfs.out rtl/neighbor_check.v rtl/bfs_engine.v tb/tb_bfs_engine.v && vvp sim/bfs.out
iverilog -g2012 -Wall -s tb_nexys3_hil -o sim/hil.out rtl/neighbor_check.v rtl/bfs_engine.v rtl/eth_udp_tx.v rtl/eth_hil_rx.v rtl/hil_brain.v rtl/nexys3_top_hil.v tb/tb_nexys3_hil.v && vvp sim/hil.out
```

Files: `rtl/` (BFS engine, brain, receiver, transmitter, top), `tb/`, `ref/` (golden Python model, packets, vector tools),
`bridge/` (UDP link, mock FPGA), `webots/` (8x8 arena world and controllers), `docs/packets.md`, `constraints/nexys3_hil.ucf`.
