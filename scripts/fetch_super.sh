#!/usr/bin/env bash
# 把 hku-mars/SUPER 拉下来。直连 github.com 超时，所以按可达性排序试三条路：
#   ① codeload.github.com 的 tarball（实测可达，301/200）
#   ② ghfast.top 代理 clone（实测 200）
#   ③ gitee 镜像（实测 200）
set -u
DEST=/home/fanxin/super_super_apm/upstream
mkdir -p "$DEST"
TMP=/tmp/superfetch
rm -rf "$TMP"; mkdir -p "$TMP"

echo "== ① codeload tarball =="
for ref in master main; do
  url="https://codeload.github.com/hku-mars/SUPER/tar.gz/refs/heads/$ref"
  f="$TMP/SUPER-$ref.tar.gz"
  code=$(timeout 120 curl -sL -o "$f" -w '%{http_code}' "$url" 2>/dev/null)
  size=$(stat -c %s "$f" 2>/dev/null || echo 0)
  echo "  $ref : http=$code size=$size"
  if [ "$size" -gt 100000 ]; then
    echo "  -> extracting"
    tar -xzf "$f" -C "$TMP"
    src=$(find "$TMP" -maxdepth 1 -type d -name 'SUPER-*' | head -1)
    if [ -n "$src" ]; then
      rm -rf "$DEST/SUPER"; mv "$src" "$DEST/SUPER"
      echo "  OK: $DEST/SUPER"
      exit 0
    fi
  fi
done

echo "== ② ghfast.top clone =="
rm -rf "$TMP/SUPER_git"
if timeout 300 git clone --depth 1 https://ghfast.top/https://github.com/hku-mars/SUPER.git "$TMP/SUPER_git" 2>&1 | tail -3; then
  if [ -d "$TMP/SUPER_git/.git" ]; then
    rm -rf "$DEST/SUPER"; mv "$TMP/SUPER_git" "$DEST/SUPER"
    echo "  OK: $DEST/SUPER"
    exit 0
  fi
fi

echo "== ③ gitee mirror search hint =="
echo "  (需要手工确认镜像名，见下一步)"
echo "FAILED: all routes exhausted"
exit 1
