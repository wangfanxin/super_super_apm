#!/usr/bin/env bash
# Did the pub_step=2 + empty-cloud-guard mitigation remove the crash, or only reduce it?
set -u
CORE=$(ls -t /tmp/dsh_core.super_lio_node.* 2>/dev/null | head -1)
NODE=/home/fanxin/lio_avoid/lio_ws/install/super_lio/lib/super_lio/super_lio_node
echo "== newest core: $CORE"
ls -la "$CORE" 2>/dev/null
echo
if [ -n "${CORE:-}" ]; then
  gdb -batch -q -ex "set pagination off" -ex "bt 8" "$NODE" "$CORE" 2>&1 \
    | grep -vE '^warning: Can|New LWP|Thread debugging|Using host' | head -18
fi
