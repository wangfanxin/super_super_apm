#!/usr/bin/env bash
# rtf trend across sorties: is the host getting slower?  H1 is a gate (0.98-1.02) and this stack's
# outcome swings on it, so the number has to be looked at, not assumed.
set -u
for t in "$@"; do
  f=/tmp/hunt-$t.log
  [ -f "$f" ] || f=/tmp/accept-$t.log
  [ -f "$f" ] || f=/tmp/accept-${t}.log
  v="-"
  if [ -f "$f" ]; then
    v=$(grep -o "rtf [0-9.]*" "$f" | head -1 | awk '{print $2}')
  fi
  printf '%-16s rtf=%s\n' "$t" "${v:--}"
done
