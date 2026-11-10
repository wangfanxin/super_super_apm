#!/usr/bin/env bash
# Set up (and PROVE) core dumps for the LIO, so the next SIGFPE can be read with gdb.
#
# The crash happens in 5 % of sorties and the node is a release build of Super-LIO, so the only honest
# way to name the line is a backtrace.  WSL ships `core_pattern = |/wsl-capture-crash ...` and the
# login shell has `ulimit -c 0`, i.e. nothing is ever written today.
set -u
echo "== before"
cat /proc/sys/kernel/core_pattern
ulimit -c
echo
echo "== 1) point core_pattern at a plain file (needs root; passwordless sudo is available)"
echo '|/usr/share/apport/apport %p %s %c %d %P' >/dev/null 2>&1 || true
sudo -n sh -c 'echo /tmp/dsh_core.%e.%p > /proc/sys/kernel/core_pattern' || echo "   (core_pattern not changed)"
cat /proc/sys/kernel/core_pattern
echo
echo "== 2) prove the plumbing with a deliberate integer divide-by-zero"
T=/tmp/dsh_core_test
cat > "$T.c" <<'EOF'
int main(void) {
    volatile int zero = 0;
    volatile int boom = 1 / zero;   /* integer division by zero -> SIGFPE */
    return boom;
}
EOF
gcc -O0 -o "$T" "$T.c" 2>/dev/null || { echo "   no gcc"; exit 1; }
( ulimit -c unlimited; cd /tmp && "./$(basename $T)" ; echo "   exit=$?" )
echo
echo "== 3) was a core written?"
ls -la /tmp/dsh_core.* 2>/dev/null | head -5
CORE=$(ls -t /tmp/dsh_core.* 2>/dev/null | head -1)
if [ -n "${CORE:-}" ]; then
  echo "== 4) gdb backtrace from $CORE"
  gdb -batch -q -ex "bt" "$T" "$CORE" 2>&1 | head -12
else
  echo "   no core file -> plumbing failed"
fi
rm -f /tmp/dsh_core.* "$T" "$T.c"
