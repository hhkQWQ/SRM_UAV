# 五组代表性仿真日志

整理日期：2026-11-19。按定位距离、相机外参、悬停结果和故障诊断价值，从 8 次运行中保留以下 5 组。

每组以一次仿真运行计数，包含 1 个 CSV、同名配置 JSON，以及配套算法和桥接日志。第 04、05 组另有同步诊断 JSONL。共 5 个 CSV、5 个 JSON、5 份算法日志、5 份桥接日志和 2 个 JSONL，另附本文。

| 主日志 | 代表内容 | 记录数 / 有效 PnP 数 | 记录时长 | 保留依据 |
|---|---|---:|---:|---|
| [pnp_sim_01_远距离观测_旧相机外参.csv](pnp_sim_01_远距离观测_旧相机外参.csv) | 远距离约 2.89 m 的定位与跟踪对照 | 1544 / 1017 | 203.3 s | 距离误差较明显，并包含预测、重新确认和丢靶；保留远距离边界案例。 |
| [pnp_sim_02_实机外参_定位观测.csv](pnp_sim_02_实机外参_定位观测.csv) | 采用实机安装外参的约 1.96 m 仿真观测 | 939 / 683 | 115.1 s | 与当前安装方向和位移一致的定位对照，包含稳定观测与移动。 |
| [pnp_sim_03_旧Mission_悬停后同步异常.csv](pnp_sim_03_旧Mission_悬停后同步异常.csv) | 旧 Mission 接管、约 1 m 悬停及后续同步异常 | 4709 / 4037 | 572.4 s | 最长完整流程：84.668 s 进入 HOLD，140.300 s 进入 TIME_UNSYNCED；保留异常后的长时间记录。 |
| [pnp_sim_04_旧Mission_稳定悬停_同步正常.csv](pnp_sim_04_旧Mission_稳定悬停_同步正常.csv) | 旧 Mission 稳定悬停与正常同步诊断 | 1115 / 518 | 135.6 s | 119.036 s 进入 HOLD 并持续至记录末尾；同步诊断中有 133 次 ACCEPT_FILTER、无同步超时事件。 |
| [pnp_sim_05_旧Mission_丢靶退出_同步抖动.csv](pnp_sim_05_旧Mission_丢靶退出_同步抖动.csv) | 旧 Mission 丢靶退出及同步抖动诊断 | 888 / 357 | 108.7 s | 73.836 s 进入 LOST_RELEASE；同步诊断有 30 次 REJECT_JITTER、4 次超时，适合与第 04 组对照。 |

记录时长为最后与最先采样时刻之差。观测距离取有效 PnP 样本的相机真值深度中位数，并非整段固定高度。

第 01 组使用旧仿真相机外参；第 02–05 组使用机体前移 14 cm、下移 4 cm 的实机安装外参。这里的“实机外参”指仿真采用该安装参数，并非实机飞行。

第 03–05 组属于已经移除的旧 Mission 流程，因此文件名明确保留“旧Mission”。全部记录采用历史字段，缺少当前 control_track，不作为当前人工切 Offboard 流程已验证通过的证据。当前导航分析会提示 CONTROL_INPUT_NOT_RECORDED。

## 重命名对应关系

| 原文件名 | 现文件名 |
|---|---|
| `pnp_sim_20261115_171800_654707.csv` | `pnp_sim_01_远距离观测_旧相机外参.csv` |
| `pnp_sim_20261115_171800_654707.json` | `pnp_sim_01_远距离观测_旧相机外参.json` |
| `algorithm_20261115_171752.log` | `algorithm_01_远距离观测_旧相机外参.log` |
| `bridge_20261115_171752.log` | `bridge_01_远距离观测_旧相机外参.log` |
| `pnp_sim_20261115_174901_222479.csv` | `pnp_sim_02_实机外参_定位观测.csv` |
| `pnp_sim_20261115_174901_222479.json` | `pnp_sim_02_实机外参_定位观测.json` |
| `algorithm_20261115_174853.log` | `algorithm_02_实机外参_定位观测.log` |
| `bridge_20261115_174853.log` | `bridge_02_实机外参_定位观测.log` |
| `pnp_sim_20261115_185928_584840.csv` | `pnp_sim_03_旧Mission_悬停后同步异常.csv` |
| `pnp_sim_20261115_185928_584840.json` | `pnp_sim_03_旧Mission_悬停后同步异常.json` |
| `algorithm_20261115_185920.log` | `algorithm_03_旧Mission_悬停后同步异常.log` |
| `bridge_20261115_185920.log` | `bridge_03_旧Mission_悬停后同步异常.log` |
| `pnp_sim_20261117_232734_711131.csv` | `pnp_sim_04_旧Mission_稳定悬停_同步正常.csv` |
| `pnp_sim_20261117_232734_711131.json` | `pnp_sim_04_旧Mission_稳定悬停_同步正常.json` |
| `algorithm_20261117_232726.log` | `algorithm_04_旧Mission_稳定悬停_同步正常.log` |
| `bridge_20261117_232726.log` | `bridge_04_旧Mission_稳定悬停_同步正常.log` |
| `pnp_sim_20261117_232734_711131_sync.jsonl` | `pnp_sim_04_旧Mission_稳定悬停_同步正常_sync.jsonl` |
| `pnp_sim_20261117_233046_926687.csv` | `pnp_sim_05_旧Mission_丢靶退出_同步抖动.csv` |
| `pnp_sim_20261117_233046_926687.json` | `pnp_sim_05_旧Mission_丢靶退出_同步抖动.json` |
| `algorithm_20261117_233039.log` | `algorithm_05_旧Mission_丢靶退出_同步抖动.log` |
| `bridge_20261117_233039.log` | `bridge_05_旧Mission_丢靶退出_同步抖动.log` |
| `pnp_sim_20261117_233046_926687_sync.jsonl` | `pnp_sim_05_旧Mission_丢靶退出_同步抖动_sync.jsonl` |

所有保留日志的文件内容与修改时间均保持不变。原始 JSON 和文本中的历史路径保留为当时运行证据；定位现有配套文件时使用上表。CSV 与配置 JSON 继续同名，CSV 保留 pnp_sim_ 前缀，诊断保留 _sync.jsonl 后缀，兼容现有分析工具。

## 已清理的重复记录

- `20261115_171114_794104`：旧外参近距离观测，与所保留的实机外参定位观测重叠；保留更有差异的远距离案例。
- `20261115_185627_262901`：短时丢靶退出案例；第 05 组同样覆盖丢靶退出，并额外提供同步诊断。
- `20261117_184053_101374`：同步异常但没有独立同步诊断；第 03 组覆盖悬停后异常，第 05 组提供更完整诊断证据。

## 分析方法

在新仿真尚未产生日志时，--latest 会选择第 05 组这一最新历史记录；新日志写入后会按修改时间选择新日志。

```bash
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_log.py --latest --last-seconds 20
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_navigation.py --latest
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_sync.py --latest
```

也可把 --latest 替换为上表主日志的完整 CSV 路径；同步分析应指定对应的 JSONL 路径。
