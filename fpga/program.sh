#!/bin/bash
# Mac side: load a bitstream into the Nexys 3 (volatile JTAG configuration, not flash) through the VM.
#   fpga/program.sh [artifacts/ff_top.bit]
# Prints PROGRAM_OK only if iMPACT reports a successful program (DONE pin high); evidence goes to logs/ff-program.log.
set -e
cd "$(dirname "$0")/.."
BIT="${1:-artifacts/ff_top.bit}"
SSH="ssh -i $HOME/.ssh/utm_fpga_ecdsa -o IdentitiesOnly=yes -o BatchMode=yes -o LogLevel=ERROR aknahin@192.168.64.2"
SCP="scp -O -q -i $HOME/.ssh/utm_fpga_ecdsa -o IdentitiesOnly=yes -o LogLevel=ERROR"
$SSH 'mkdir -p ~/ff-program'
$SCP "$BIT" aknahin@192.168.64.2:ff-program/top.bit
$SSH 'cat > ~/ff-program/program.cmd <<CMD
setMode -bs
setCable -port auto
identify
assignFile -p 1 -file top.bit
program -p 1
quit
CMD'
$SSH 'bash -c "set --; source /opt/Xilinx/14.6/ISE_DS/settings64.sh >/dev/null; cd ~/ff-program; impact -batch program.cmd"' 2>&1 | tee logs/ff-program.log | tail -8
# The Ethernet PHY needs a few seconds to reset and negotiate after configuration; frames sent earlier are
# lost, which desynchronises the brain from the reference (seen once: 647 timeouts right after programming).
if grep -q "Programmed successfully" logs/ff-program.log; then sleep 10; echo PROGRAM_OK; else echo PROGRAM_FAILED; exit 1; fi
