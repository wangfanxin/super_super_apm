# 两条代表路线的复现命令（2026-11-27）

前置：仓库根目录，`bash scripts/clean_harness.sh` 之后执行；世界固定 `sim/worlds/forest_hex12.yaml`（522 柱）。

## A 路线：15 行全过、质量最好（角点会停-转，真值 0.02-0.12 m/s）
```
SITL_MODEL="x:sim/models/quad_1kg_250mm.json" \
EXTRA_PARMS=$PWD/fcu_interface/params/guided_limits_bench5_vert_agile.parm \
GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=0.8 ARRIVE_M=0.3 PRE_GOAL_M=0.8 \
REF_NO_RETRACT=1 REF_Z_OFFSET_M=0.118 REF_Z_HOLD=2.0 REF_Z_TOL=0.005 REF_Z_LATCH_M=1.2 \
NAV_GATE_MPS=15 NAV_SKIP_MAX=40 WARMUP_SECS=90 BAG=1 BAG_SPLIT=1 SORTIES=1 \
bash scripts/fly_hex2.sh phase44b 1
python3 scripts/quality_rows.py --tag phase44b-01-01     # Q1 1 段, Q3 0.193 PASS, W4 +0.037, W6 0.259
```

## B 路线：指令速度全程 ≥1 m/s、角点基本不停车（真值 0.4-0.8，但 W4 接触）
```
SITL_MODEL="x:sim/models/quad_1kg_250mm.json" \
EXTRA_PARMS=$PWD/fcu_interface/params/guided_limits_bench5_vert_agile.parm \
GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=1.0 ARRIVE_M=0.3 \
REF_NO_RETRACT=1 REF_GOV_MIN=1.0 REF_V_SLEW=3.0 \
REF_Z_OFFSET_M=0.118 REF_Z_HOLD=2.0 REF_Z_TOL=0.005 REF_Z_LATCH_M=1.2 \
NAV_GATE_MPS=15 NAV_SKIP_MAX=40 WARMUP_SECS=90 BAG=1 BAG_SPLIT=1 SORTIES=1 \
bash scripts/fly_hex2.sh phase66a 1
python3 scripts/quality_rows.py --tag phase66a-01-01     # 指令 min/p1 1.00; 角点真值 0.56/0.70/0.10/0.68/0.75/0.64
```

## 关键实现（本轮新增/修复）
- `--ref-gov-min-mps`（调速器最小速度下限；`REF_GOV_MIN=1.0` 是指令全程 ≥1 m/s 的关键）
- 修复 `REF_V_SLEW` 从未生效的 dt bug（`last_pub_wall` → `last_pub_t`）
- `--ref-no-retract` / `--ref-no-fold` / `--ref-retract-dr` / `--ref-handover-gate` / `--ref-final-slew` 等交接层开关
- SUPER 侧：patch 19（延迟交接）、patch 20（自适应出弯延长）、patch 21（guide 圆角）、`hot_start_vel_ratio` 配置
- 验收：`bash tests/run_all.sh`（GATE: PASS）；`scripts/accept.py` 15 行 + `scripts/quality_rows.py` Q1-Q4
