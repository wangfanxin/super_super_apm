# 文档索引

按"读它的目的"分组，并标注状态：**current**（当前实现，以此为准）、**deep-dive**（当前实现的细节）、
**record**（历史测量记录，不改写）、**historical**（旧阶段的笔记，已被取代）。

| 文档 | 状态 | 什么时候读它 |
|---|---|---|
| [DATA_PIPELINE.md](DATA_PIPELINE.md) | **current** | 想知道数据从哪来、经过谁、变成什么、怎么评分 —— 先读这一份 |
| [SPEC.md](SPEC.md) | **current** | 任务/地图/速度/机体参数的硬规矩（用户口述，逐条固化，含修改记录表） |
| [CHAIN_DETAIL.md](CHAIN_DETAIL.md) | **deep-dive** | 航点盒、SUPER FSM、薄层每拍判断的逐行注解 |
| [INTEGRATION.md](INTEGRATION.md) | **record** | 9000+ 行的测量记录：试过什么、数字、为什么否掉（§99–§112 是第 20 轮） |
| [PIPELINE-PLAN.md](PIPELINE-PLAN.md) | **proposal** | 数据管线完善方案（分阶段、含验证与回滚）——待所有者批准后执行 |
| [PATHS.md](PATHS.md) | **current** | 路径约定：仓库内 vs 运行根 vs 历史路径（读旧记录时对照） |
| [evidence/README.md](evidence/README.md) | **record** | 四张证据图是什么、怎么重画 |
| [CHAIN.md](CHAIN.md) | historical | 第 87 轮的逐段 topic/频率快照 |
| [PIPELINE.md](PIPELINE.md) | historical | 更早的数据链路与状态机笔记（以 `missionA-pe-01` 为准） |
| [MISSION.md](MISSION.md) | historical | 15 m 正方形 × 3 圈的旧场地说明（现行是六边形，见 SPEC S0/S2） |
| [README-integration-history.md](README-integration-history.md) | historical | 第 88 轮之前的项目 README 原文 |

规则：**current 文档以代码为准**；发现文档与代码不一致时，先改文档，再在 INTEGRATION.md 里记一条。
