#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# What exactly divided by zero inside pcl::transformPointCloud?  Registers + disassembly at $pc,
# plus the OpenMP environment (PCL splits the loop with omp_get_max_threads()).
set -u
CORE=/tmp/dsh_core.super_lio_node.5599
NODE=${LIO_WS}/install/super_lio/lib/super_lio/super_lio_node
echo "== OMP environment in this shell"
env | grep -iE '^OMP|^OMPI' || echo "   (none)"
echo "== what the harness passes"
grep -rn 'OMP_NUM_THREADS' "${DSH_ROOT}/harness" "${DSH_ROOT}" --include=*.sh --include=*.py --include=*.bash 2>/dev/null | head -5
echo
echo "== disassembly around the faulting instruction"
gdb -batch -q \
    -ex "set pagination off" \
    -ex "x/14i \$pc-32" \
    -ex "echo \n--- registers ---\n" \
    -ex "info registers rax rbx rcx rdx rsi rdi r8 r9 r10 r11" \
    "$NODE" "$CORE" 2>&1 | grep -v 'warning: Can' | tail -30
