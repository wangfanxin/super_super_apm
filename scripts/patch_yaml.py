#!/usr/bin/env python3
"""Path-aware, comment-preserving YAML value setter.

Why not PyYAML: the SUPER configs carry the experiment record in their comments
("this value came from run X, do not hand-edit"), and a load/dump cycle destroys
them.  Why not sed: `enable:` / `resolution:` appear under several parents
(esdf, map_sliding, ros_callback, visualization), so a flat substitution silently
edits the wrong block.  This walks the file by indentation, resolves the *full*
dotted path of every scalar key, and rewrites only the match.

Usage:
  patch_yaml.py FILE --set rog_map.visualization.enable=false [--set a.b=1 ...]
  patch_yaml.py FILE --get rog_map.visualization.enable
  patch_yaml.py FILE --dump            # every scalar as dotted.path=value
"""
import sys


def scan(lines):
    """Return [(line_index, dotted_path, indent, key, value_text, had_value)]."""
    out = []
    stack = []           # [(indent, key)]
    for i, raw in enumerate(lines):
        stripped = raw.split("#", 1)[0].rstrip("\r\n")
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip(" "))
        body = stripped.strip()
        if body.startswith("-"):
            continue                      # list item: not addressable here
        if ":" not in body:
            continue
        key, rest = body.split(":", 1)
        key = key.strip()
        if not key or " " in key:
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = ".".join([k for _, k in stack] + [key])
        value = rest.strip()
        had = bool(value)
        out.append((i, path, indent, key, value, had, raw))
        stack.append((indent, key))
    return out


def fmt_value(v):
    low = v.lower()
    if low in ("true", "false", "null", "~"):
        return low
    try:
        float(v)
        return v
    except ValueError:
        pass
    v = v.strip()
    if v.startswith("[") and v.endswith("]"):
        return v
    if (v.startswith('"') and v.endswith('"')) or (v.startswith("'") and v.endswith("'")):
        return v
    return '"%s"' % v


def set_key(lines, path, value):
    found = [e for e in scan(lines) if e[1] == path]
    if not found:
        raise SystemExit("no such key: %s" % path)
    if len(found) > 1:
        raise SystemExit("ambiguous key %s: %d matches" % (path, len(found)))
    i, _, indent, key, _, _, raw = found[0]
    cr = "\r\n" if raw.endswith("\r\n") else "\n"
    tail = ""
    if "#" in raw:
        tail = "   #" + raw.split("#", 1)[1].rstrip("\r\n")
    lines[i] = "%s%s: %s%s%s" % (" " * indent, key, fmt_value(value), tail, cr)
    return i


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    path = sys.argv[1]
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    mode, sets = None, []
    for a in sys.argv[2:]:
        if a in ("--dump", "--get"):
            mode = a
        elif a.startswith("--set"):
            continue
        elif a.startswith("--"):
            raise SystemExit("unknown option %s" % a)
        elif mode == "--get":
            sets.append(a)
        else:
            sets.append(a)

    if "--dump" in sys.argv[2:]:
        for _, p, _, _, v, _, _ in scan(lines):
            print("%s=%s" % (p, v))
        return

    changed = 0
    for item in sets:
        if "=" not in item:
            if "--get" in sys.argv[2:]:
                for _, p, _, _, v, _, _ in scan(lines):
                    if p == item:
                        print("%s=%s" % (p, v))
                continue
            raise SystemExit("expected path=value, got %r" % item)
        p, v = item.split("=", 1)
        i = set_key(lines, p.strip(), v.strip())
        print("  line %-4d %s = %s" % (i + 1, p.strip(), fmt_value(v.strip())))
        changed += 1

    if changed:
        with open(path, "w", encoding="utf-8") as f:
            f.writelines(lines)
    print("%s: %d value(s) written" % (path, changed))


main()
