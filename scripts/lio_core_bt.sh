#!/usr/bin/env bash
# The core from kj1-01-01's LIO (SIGFPE).  Print the faulting line and the stack.
set -u
CORE=/tmp/dsh_core.super_lio_node.5599
NODE=/home/fanxin/lio_avoid/lio_ws/install/super_lio/lib/super_lio/super_lio_node
echo "== core: $CORE"
ls -la "$CORE" 2>/dev/null
echo "== node: $NODE"
ls -la "$NODE" 2>/dev/null
echo
echo "== gdb: where did it die?"
gdb -batch -q \
    -ex "set pagination off" \
    -ex "bt 25" \
    -ex "echo \n--- faulting frame ---\n" \
    -ex "frame 0" \
    -ex "info registers rax rbx rcx rdx rsi rdi" \
    -ex "info line *\$pc" \
    "$NODE" "$CORE" 2>&1 | head -60
