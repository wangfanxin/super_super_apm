#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Did the pub_step=2 + empty-cloud-guard mitigation remove the crash, or only reduce it?
set -u
CORE=$(ls -t /tmp/dsh_core.super_lio_node.* 2>/dev/null | head -1)
NODE=${LIO_WS}/install/super_lio/lib/super_lio/super_lio_node
echo "== newest core: $CORE"
ls -la "$CORE" 2>/dev/null
echo
if [ -n "${CORE:-}" ]; then
  gdb -batch -q -ex "set pagination off" -ex "bt 8" "$NODE" "$CORE" 2>&1 \
    | grep -vE '^warning: Can|New LWP|Thread debugging|Using host' | head -18
fi
