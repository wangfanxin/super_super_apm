# tests/ —— 无飞行自检

```bash
bash tests/run_all.sh            # 全部
bash tests/run_all.sh --only-syntax   # 只做语法（CI 用）
```

| 步骤 | 检查什么 | 失败意味着 |
|---|---|---|
| `bash -n` / `py_compile` | 每个 .sh / .py 能否解析 | 有语法错误 |
| `scripts/check_static.sh` | SPEC 自检（S0/S1/S2/S4）+ 冻结地图 + 两个上游补丁在位 | 配置/世界/源码被改坏 |
| `scripts/check_pattern.sh` | 传感器射线图案存在且形状正确 | 飞行会拒绝启动 |
| `test_ref_lp.py` / `test-stall-gate.py` | 薄层低通、H6 闸门的单元测试 | 行为回退 |
| `tests/no_abs_paths.sh` | 飞行/评分链不含作者绝对路径 | 仓库不再可移植 |
| `scripts/check_evidence_links.sh` | `docs/evidence/` 与文档引用一致 | 图与文档脱节 |
| 工具 `--help` 冒烟 | `scripts/*.py` 至少能被解析成命令行程序 | 有工具导入即崩 |
