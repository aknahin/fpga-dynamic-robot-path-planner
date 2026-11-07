#!/bin/bash
# Mac side: copy the design to the VM, build it with ISE, fetch the reports.
#   fpga/ise.sh synth     synthesis only (fast)
#   fpga/ise.sh all       synth + implement + bitstream, copies artifacts/ and logs/
set -e
cd "$(dirname "$0")/.."
SSH="ssh -i $HOME/.ssh/utm_fpga_ecdsa -o IdentitiesOnly=yes -o BatchMode=yes -o LogLevel=ERROR aknahin@192.168.64.2"
SCP="scp -O -q -i $HOME/.ssh/utm_fpga_ecdsa -o IdentitiesOnly=yes -o LogLevel=ERROR"
$SSH 'rm -rf ~/ff-build && mkdir -p ~/ff-build/rtl ~/ff-build/constraints'
$SCP rtl/ff_tables.v rtl/eth_udp_tx.v rtl/eth_ff_rx.v rtl/ff_brain.v rtl/nexys3_top_ff.v aknahin@192.168.64.2:ff-build/rtl/
$SCP constraints/nexys3_ff.ucf aknahin@192.168.64.2:ff-build/constraints/
$SCP fpga/build_ff.sh aknahin@192.168.64.2:ff-build/
mkdir -p artifacts logs
$SSH "bash ~/ff-build/build_ff.sh ${1:-all}" 2>&1 | tee logs/ff-build-$1.log | tail -5
$SCP aknahin@192.168.64.2:ff-build/build/synthesis.log logs/ff-synthesis.log || true
if [ "$1" != synth ]; then
  for f in top.bit timing.twr top.par top.mrp top.bld; do
    $SCP aknahin@192.168.64.2:ff-build/build/$f artifacts/ff_$f 2>/dev/null || true
  done
fi
