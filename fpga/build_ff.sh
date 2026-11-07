#!/bin/bash
# Runs inside the Ubuntu VM (ISE 14.6). Builds nexys3_top_ff.bit from ~/ff-build/{rtl,constraints}.
# Usage: bash build_ff.sh [synth|all]
set -e
MODE=${1:-all}
set --   # settings64.sh would read our arguments as its own
source /opt/Xilinx/14.6/ISE_DS/settings64.sh >/dev/null
cd "$(dirname "$0")"
mkdir -p build && cd build
printf 'verilog work ../rtl/ff_tables.v\nverilog work ../rtl/eth_udp_tx.v\nverilog work ../rtl/eth_ff_rx.v\nverilog work ../rtl/ff_brain.v\nverilog work ../rtl/nexys3_top_ff.v\n' > top.prj
cat > top.xst <<'XST'
run
-ifn top.prj
-ifmt mixed
-ofn top.ngc
-ofmt NGC
-p xc6slx16-3-csg324
-top nexys3_top_ff
-opt_mode Speed
-opt_level 1
XST
xst -ifn top.xst -ofn synthesis.log
[ "$MODE" = synth ] && exit 0
ngdbuild -p xc6slx16-3-csg324 -uc ../constraints/nexys3_ff.ucf top.ngc top.ngd
map -w -p xc6slx16-3-csg324 -o top_map.ncd top.ngd top.pcf
par -w top_map.ncd top.ncd top.pcf
trce -v 10 -o timing.twr top.ncd top.pcf
bitgen -w -g StartUpClk:JtagClk -g UnusedPin:Pullnone top.ncd top.bit top.pcf
