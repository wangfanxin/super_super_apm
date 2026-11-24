#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# The core from kj1-01-01's LIO (SIGFPE).  Print the faulting line and the stack.
set -u
CORE=/tmp/dsh_core.super_lio_node.5599
NODE=${LIO_WS}/install/super_lio/lib/super_lio/super_lio_node
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
