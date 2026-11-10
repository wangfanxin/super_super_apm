#!/usr/bin/env bash
# What exactly divided by zero inside pcl::transformPointCloud?  Registers + disassembly at $pc,
# plus the OpenMP environment (PCL splits the loop with omp_get_max_threads()).
set -u
CORE=/tmp/dsh_core.super_lio_node.5599
NODE=/home/fanxin/lio_avoid/lio_ws/install/super_lio/lib/super_lio/super_lio_node
echo "== OMP environment in this shell"
env | grep -iE '^OMP|^OMPI' || echo "   (none)"
echo "== what the harness passes"
grep -rn 'OMP_NUM_THREADS' /mnt/d/distill_self/lio_avoid /mnt/d/distill_self/super_super_apm --include=*.sh --include=*.py --include=*.bash 2>/dev/null | head -5
echo
echo "== disassembly around the faulting instruction"
gdb -batch -q \
    -ex "set pagination off" \
    -ex "x/14i \$pc-32" \
    -ex "echo \n--- registers ---\n" \
    -ex "info registers rax rbx rcx rdx rsi rdi r8 r9 r10 r11" \
    "$NODE" "$CORE" 2>&1 | grep -v 'warning: Can' | tail -30
