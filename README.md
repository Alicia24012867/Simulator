# SPICE-like Circuit Simulator

这是一个基于 C++17 和 Eigen 的类 SPICE 电路仿真器。目前支持 DC operating point、基础 transient analysis 最小闭环，以及实验性的 pseudo-transient analysis (PTA) 路径，使用稀疏 MNA、SparseLU、Newton 迭代、步长限制和 source stepping 完成求解。瞬态分析首个积分步采用 Backward Euler，后续在步长增长不超过前一步两倍时采用可变步长 BDF2。这里的 TRAN 是可运行的 MVP，不等同于完整 SPICE 瞬态引擎。

本项目实现的是明确受限的 SPICE 子集，并非完整 ngspice 替代品。I/O 层遵循常见 SPICE netlist、`.print` listing 和 ASCII rawfile 约定，未支持的控制卡或输出表达式会直接报错，不会静默忽略。

## 代码结构

- `src/main.cpp`：只负责装配配置、解析、求解和输出流程，保持为薄入口。
- `src/app/command_line.cpp`：命令行语法、重复参数检查和帮助文本。
- `src/netlist/reader.cpp`：文件读取、注释与续行处理，以及 `.end` 位置校验。
- `src/netlist/subcircuit.cpp`：`.subckt` 定义收集和 `X` 实例递归展平；不依赖 `Circuit`，只输出原语 token。
- `src/netlist/parser.cpp`：控制卡、模型和原语器件的语义校验与构造。
- `src/config/config_loader.cpp`：`config.json` 的定位与加载。
- `src/config/parse_overrides.cpp`：严格 schema 校验，并生成类型化覆盖层。
- `src/config/apply_overrides.cpp`、`option_overrides.cpp`：分别应用配置文件覆盖和命令行 `name=value` 覆盖，同时保护网表中显式指定的参数。
- `src/circuit/`：节点编号、MNA 构建及 OP/TRAN/PTA 求解调度。
- `src/io/spice_output.cpp`：listing、ASCII rawfile 格式化，以及多输出文件的事务式写入。
- `third_party/nlohmann/json.hpp`：随仓库固定版本的 header-only JSON 解析器。

这种分层使网表语法、层次展开和求解模型可以独立演进。大型网表的普通逻辑行不再长期保留原始文本；子电路引脚索引在定义阶段预计算，展开每个实例时无需构造临时绑定表。

## 已支持功能

### 器件

| 前缀 | 器件 | OP | TRAN |
| --- | --- | --- | --- |
| `R` | Resistor | 电导 stamp | 电导 stamp |
| `C` | Capacitor | 开路 | Backward Euler / BDF2 companion model |
| `L` | Inductor | 0 V 支路 | Backward Euler / BDF2 branch equation |
| `V` | Independent voltage source | MNA branch | 当前仅支持固定 DC 值 |
| `I` | Independent current source | RHS stamp | 当前仅支持固定 DC 值 |
| `D` | Diode | 指数模型与 Newton 线性化 | 静态非线性模型 |
| `Q` | NPN / PNP BJT | 简化 Ebers-Moll 风格模型 | 静态非线性模型 |
| `M` | NMOS / PMOS | 简化 Level-1 平方律模型 | 静态非线性模型 |

`.model` 支持 `D`、`NPN`、`PNP`、`NMOS`、`PMOS`、`NCH`、`PCH`。当前求解方程使用的模型参数包括：

- Diode：`IS`, `N`, `VT`, `GMIN`
- BJT：`IS`, `BF` / `BETA`, `BR`, `NF`, `NR`, `VT`, `GMIN`，以及 DC Gummel-Poon 子集 `RB`、`RC`、`RE`、`VA` / `VAF`、`VAR`、`IKF`、`IKR`、`ISE`、`ISC`、`NE`、`NC`
- MOSFET：`LEVEL=1`，以及为遗留网表兼容读取的 `LEVEL=3` model card；求解仍使用 `VTO` / `VT0`、`KP` / `K`、`LAMBDA` / `LAM`、`GMIN` 的简化 Level-1 平方律
- 实例参数：`AREA`, `W`, `L`

读取器还接受 Diode 的 `RS`、BJT 的 `RBE` / `RCE`，以及 MOSFET 的 `RDS` 和常见 Level-3 card 参数（如 `TOX`、`LD`、`UO`、结电容参数）。后者用于读取遗留网表，尚未写入器件 stamp，求解结果仍是上述简化 Level-1 近似。未知参数、非数值参数、非物理的负值以及非 `LEVEL=1` / `LEVEL=3` 的 MOSFET model 会在读取阶段明确报错。

### Netlist 读取规则

- 第一物理行作为 circuit title；`.title ...` 可以覆盖它。
- 元件名、模型名、节点名和控制卡大小写不敏感。
- `0` 是 ground；额外接受 `gnd` 作为别名。
- 整行注释使用 `*`；行尾注释支持 `;`、`$` 和 `//`。
- `#` 不是注释符，因为 SPICE branch vector 会使用 `v1#branch` 形式。
- `+` 开头的逻辑续行会拼接到上一条语句；孤立续行会报告文件名和行号。
- 支持 `.subckt name [pin...] [params: name=value ...]` / `.ends [name]` 与 `X` 子电路实例；解析时会递归展平为现有 primitive 元件。`X` 实例可用 `PARAMS:`（可省略）后的 `name=value` 覆盖默认参数；参数值支持 `{...}` 包裹的 `+`、`-`、`*`、`/`、`^` 表达式和对上层参数的引用，并会在展平时落实为数值 token。子电路当前支持 `R`、`C`、`L`、`V`、`I`、`D`、`Q`、`M`、嵌套 `X`，不支持子电路体内的控制卡。
- `.end` 必须存在、不接受参数，并且必须是最后一个非注释语句。
- 支持数值后缀 `a`, `f`, `p`, `n`, `u`, `m`, `mil`, `k`, `meg`, `g`, `t`；其中 `m` 表示 milli，mega 必须写成 `meg`。
- 独立源接受 `5`、`DC 5`、`DC=5`、`DC= 5` 和 `DC = 5`；当前不接受任意 `key=value` 代替 DC 值。
- 数值 token 会完整校验，`1..2`、`1k=2` 等畸形写法不会只读取前缀后继续运行。
- 网表必须至少包含一个元件和一个非 ground 节点，避免把零维 MNA 系统送入求解器。
- 当前支持的控制卡为 `.title`、`.model`、`.subckt` / `.ends`、`.op`、`.tran`、`.pstran`、`.option` / `.options`、`.print` 和 `.end`。其他 dot command 会明确报错。

### 分析与 `.print`

支持：

```spice
.op
.tran TSTEP TSTOP [TSTART [TMAX]] [UIC]
.options delmax=...
.pstran convval=... initstep=... minstep=... maxstep=... [tau=... vbe0=... kvgs0=... tauramp=...]
.print op v(node) v(node1,node2) i(device)
.print tran v(node) v(node1,node2) i(device)
```

- `v(node1,node2)` 在输出层计算差分电压，不改变 MNA 方程。
- transient listing 自动把 `time` 放在第一列；`.print tran time ...` 也可以接受。
- 多条相同分析类型的 `.print` 会按出现顺序合并并去重。
- 没有 `.print` 时，默认输出所有非 ground 节点电压和所有 branch unknown 电流。
- `i(device)` 当前只适用于具有 branch unknown 的器件，即独立电压源和电感；请求其他器件电流会得到明确错误。
- 没有分析卡时默认执行 `.op`。同时存在 `.op` 与 `.tran` 时依次输出两个分析块。
- `.tran` 未指定 `UIC` 时先求 operating point；指定 `UIC` 时当前使用全零 MNA 初值。尚未支持器件 `IC=`。
- `.option DELMAX=value`（也接受 `.options`）是 HSPICE 兼容的内部时间步长硬上限，数值接受 SPICE 后缀。ngspice 的标准、可移植写法是 `.tran` 的第四个 `TMAX` 参数；两者同时出现时取更小者，确保每个内部积分步都不超过任一上限。该限制也应用于 `.pstran` 的伪时间步，且不改变 `.op`。ngspice 当前可接受 `DELMAX` 这个非标准 option 名称，但不将其列为通用 `.options` 变量；本程序刻意实现其 HSPICE 语义，而非静默忽略。
- `.pstran` 启用 PTA operating-point 求解，接受不区分大小写的 `convval`、`initstep`、`minstep`、`maxstep`、`tau`、`vbe0`、`kvgs0`、`tauramp` 参数，且可使用 `key=value`、`key = value` 或 `key= value` 写法。`convval` 映射为 PTA 的导数和 DC 残差阈值，三个 step 参数映射为 PTA 步长边界；`tau` 启用复合伪元件，`vbe0` 设置 BJT 初始结电压，`tauramp` 控制独立源斜坡。`kvgs0` 当前仅保存和校验，尚未参与求解。`.pstran` 不能与命令行 `--pta` 同时指定。
- `TSTEP` 控制输出间隔，`TSTART` 控制开始保存的时间，`TMAX` 限制内部积分步长。当前未指定 `TMAX` 时内部最大步长使用 `TSTEP`；输出时间点也会强制成为积分点，因此与 ngspice 的默认自适应步长策略不同。
- 每次瞬态分析的首步使用 Backward Euler 的 step-doubling 误差估计；之后在新步长不大于前一步两倍时使用可变步长 BDF2，并以预测—校正差估计误差。超过误差预算或 Newton 未收敛的步会回滚并缩小后重试；内部积分点仍不会越过输出时间点。

### 实验性 PTA

命令行可选择 OP 求解路径：

```sh
./spice --pta disabled input.cir  # 默认：Newton + source stepping
./spice --pta force input.cir     # 只使用 PTA
./spice --pta fallback input.cir  # 常规 OP 失败后尝试 PTA

# 使用 SPICE 数值后缀覆盖 PTA 配置；可重复指定
./spice --pta force \
  --pta-option initial-step=1n \
  --pta-option maximum-steps=20000 \
  --pta-option successful-step-scale=1.5 \
  --pta-option medium-oscillation-ratio=0.5 \
  --pta-option heavy-oscillation-ratio=1.0 \
  --pta-option include-diodes=true \
  input.cir
```

`--pta-option name=value` 只可与 `--pta force` 或 `--pta fallback` 一起使用。数值选项接受与网表相同的 SPICE 后缀；布尔选项 `include-mos-bulk` 与 `include-diodes` 接受 `true` / `false` 或 `1` / `0`。可用名称为：

- 时间与收敛：`initial-step`、`minimum-step`、`maximum-step`、`maximum-steps`、`derivative-tolerance`、`derivative-relative-tolerance`、`derivative-voltage-absolute-tolerance`、`derivative-current-absolute-tolerance`、`dc-residual-tolerance`、`dc-residual-relative-tolerance`、`dc-voltage-absolute-tolerance`、`dc-current-absolute-tolerance`
- 伪元件：`initial-node-capacitance`、`minimum-node-capacitance`、`maximum-node-capacitance`、`current-source-capacitance`、`voltage-source-inductance`
- 自适应规则：`failed-step-scale`、`successful-step-scale`、`capacitance-grow-scale`、`small-oscillation-scale`、`medium-oscillation-scale`、`heavy-oscillation-scale`、`medium-oscillation-ratio`、`heavy-oscillation-ratio`
- 放置开关：`include-mos-bulk`、`include-diodes`

同一 PTA 选项不可重复指定；所有覆盖值会在建模前统一执行配置校验，非法范围或相互矛盾的边界会以命令行错误退出。

PTA 在 MNA pattern 固化前加入人工伪元件：独立电压源 branch 上的伪电感、独立电流源两端的伪电容，以及晶体管节点到地的伪电容。其伪时间迭代复用现有的 Backward Euler / 受步长比限制的 BDF2 `TransientIntegrator`；每一步以归一化 BDF 导数和归一化原始 OP 残差共同判定稳态。导数指标为 `h*|dx/dt| / (abstol + reltol*scale)`；残差指标为 `|Ax-b| / (abstol + reltol*max(|Ax|, |b|))`。二者都会区分节点 KCL 行的电流绝对容差与电压源支路行的电压绝对容差，`derivative-tolerance` 和 `dc-residual-tolerance` 是对应的无量纲阈值。使用 `--pta-diagnostics` 可将 PTA 是否实际执行、收敛指标、全局增容次数、节点降容次数和最小步长恢复次数写至 stderr。Newton 失败时先缩小伪时间步长；在最小步长仍失败时，增大所有节点伪电容并重启积分历史。每个成功步后，伪时间步会按 `successful-step-scale` 增长，同时仍受最大步长和 BDF2 步长比限制；节点电压变化反向时会按相邻步变化幅度比降低该节点伪电容，两个 ratio 参数分别划分小/中及中/重振荡。

该功能仍处于实验阶段。自适应规则具有单元测试，并有一条端到端夹具覆盖“最小步长失败 → 全局增容 → 重启 → 恢复收敛”路径及后续节点降容。当前 Force OP 回归覆盖的 18 个网表、76 个输出值均可通过参考对比。归一化导数收敛已避免固定绝对阈值随伪时间步和未知量量级失真的问题；其默认容差与困难非线性电路的鲁棒性仍需更广泛的基准验证。因此 `force` 与 `fallback` 可用于回归和实验，但暂不视为生产求解保证。

## 输出格式

### SPICE listing

默认写到 stdout；使用位置参数或 `-o` 写入文件。`.print` 决定 listing 的变量和顺序：

```text
Circuit: Level 1 - RC step with UIC

Transient Analysis
No. of Data Rows : 11

----------------------------------------------------------------------------------------
Index   time                v(in)               v(out)              v1#branch
----------------------------------------------------------------------------------------
       0    0.0000000000e+00    0.0000000000e+00    0.0000000000e+00    0.0000000000e+00
       1    1.0000000000e-04    5.0000000000e+00    4.5454545455e-01   -4.5454545455e-03
```

branch current 在 listing 中使用 ngspice 常见的 `device#branch` 列名。

### ASCII rawfile

使用 `-r` 额外写出 SPICE ASCII rawfile。rawfile 不受 `.print` 过滤，包含全部节点电压和全部 branch unknown 电流；瞬态的第一个 variable 固定为 `time`。

```text
Title: Level 1 - RC step with UIC
Date: Tue Jul 14 12:00:00 2026
Plotname: Transient Analysis
Flags: real
No. Variables: 4
No. Points: 11
Variables:
	0	time	time
	1	v(in)	voltage
	2	v(out)	voltage
	3	i(v1)	current
Values:
 0	0.000000000000000e+00
	0.000000000000000e+00
	0.000000000000000e+00
	0.000000000000000e+00
```

输出数值使用 classic locale、科学计数法，并拒绝写出 NaN/Inf。程序先在内存中生成结果，再把所有文件输出写入目标目录中的临时文件；全部暂存成功后才备份并替换旧文件，正常写入错误会触发回滚。解析、构建、求解或暂存失败都不会提前截断旧结果。input、listing 和 rawfile 不能通过规范路径、符号链接或硬链接指向同一文件。

## 构建与运行

依赖：

- C++17 编译器
- Eigen3
- Python 3（测试脚本）
- Make
- ngspice 46（仅在重新生成独立参考结果时需要；普通构建与 `make test` 不需要）

macOS 可安装 Eigen：

```sh
brew install eigen
```

构建：

```sh
make
```

运行并输出 listing：

```sh
./spice tests/cases/op/level1_01_resistive_bridge_mesh.cir
./spice tests/cases/op/level1_01_resistive_bridge_mesh.cir result.out
./spice -b -o result.out tests/cases/op/level1_01_resistive_bridge_mesh.cir
./spice --parse-only tests/private/UA741PFBx10.sp
```

同时生成 listing 与 rawfile：

```sh
./spice -b -o result.out -r result.raw tests/cases/tran/level1_01_rc_step_ladder.cir
```

### 配置文件发现与校验

程序可读取名为 `config.json` 的 JSON 配置文件，用于注入 OP、PTA 和 TRAN 的求解参数。不存在配置文件时，全部默认值与此前保持一致。网表控制卡中显式给出的同名参数始终优先于配置文件。

默认会从进程的当前工作目录开始查找 `config.json`，再逐级查找父目录；默认最多向上查找 8 个父目录。找到最近的文件后停止搜索。注意搜索起点是启动 `spice` 时的工作目录，而不是网表文件所在的目录。

```sh
# 自动搜索，并打印实际使用的配置文件（写到 stderr）
./spice --print-config-path tests/cases/op/level1_01_resistive_bridge_mesh.cir

# 只读取指定配置文件；指定路径不存在或不可读会以参数错误退出
./spice --config ./config.json \
  --print-config-path \
  tests/cases/op/level1_01_resistive_bridge_mesh.cir

# 只检查当前工作目录，不查询父目录
./spice --config-search-depth 0 \
  --print-config-path \
  --parse-only tests/private/UA741PFBx10.sp
```

- `--config <path>`：只读取该路径，不再自动搜索。相对路径相对于当前工作目录。
- `--config-search-depth <N>`：自动搜索时最多向父目录移动 `N` 次；`0` 表示只检查当前工作目录。
- `--print-config-path`：向 stderr 输出实际选中的配置文件；未找到时输出内建默认值正在生效。
- 自动搜索未找到配置文件不是错误；显式路径不存在、路径不是普通文件、JSON 非法或 JSON 根节点不是对象时，程序以退出码 2 失败。

配置根节点必须包含 `schema_version: 1`。物理量既可以写 JSON 数值，也可以写 SPICE 数值字符串（如 `"2n"`、`"100u"`）。例如：

```json
{
  "schema_version": 1,
  "op": {
    "newton": {
      "maximum_iterations": 1000,
      "tolerance": "1n",
      "maximum_solution_step": 1.0
    },
    "source_stepping": {
      "enabled": true,
      "initial_step": 0.1,
      "maximum_step": 0.25,
      "minimum_step": "100u",
      "growth_factor": 1.5,
      "failure_scale": 0.5
    }
  },
  "pta": {
    "mode": "fallback",
    "newton": {"maximum_iterations": 1000},
    "initial_step": "1n",
    "maximum_steps": 10000
  },
  "tran": {
    "output_interval": "1n",
    "stop_time": "100n",
    "solver": {
      "newton": {"tolerance": "1n"},
      "relative_tolerance": 0.0001
    }
  }
}
```

允许的字段如下；未知字段、错误类型、无穷数和非法 SPICE 数值都会以配置错误退出。

- `op.newton`：`maximum_iterations`、`tolerance`、`maximum_solution_step`。
- `op.source_stepping`：`enabled`、`initial_step`、`maximum_step`、`minimum_step`、`growth_factor`、`failure_scale`。
- `pta.newton`：与 `op.newton` 相同。`pta` 还支持 `mode`、`initial_step`、`minimum_step`、`maximum_step`、`maximum_steps`、所有 `derivative_*` 与 `dc_*` 容差、`initial_node_capacitance`、`minimum_node_capacitance`、`maximum_node_capacitance`、`current_source_capacitance`、`voltage_source_inductance`、`compound_time_constant`、`compound_initial_resistance`、`compound_initial_conductance`、`source_ramp_time`、`initial_bjt_vbe`、所有振荡/电容缩放字段，以及 `include_mos_bulk`、`include_diodes`。
- `tran`：`enabled`、`output_interval`、`stop_time`、`output_start_time`、`maximum_step`、`use_initial_conditions`。
- `tran.solver.newton`：与 `op.newton` 相同。`tran.solver` 还支持 `relative_tolerance`、`voltage_absolute_tolerance`、`current_absolute_tolerance`、`minimum_step`、`safety_factor`、`minimum_scale`、`maximum_scale`、`convergence_failure_scale` 和 `maximum_rejects`。

### 命令行分析参数覆盖

在读取并应用 `config.json` 后，可以使用下列可重复的选项覆盖任意分析参数：

```sh
--op-option <name=value>
--pta-option <name=value>
--tran-option <name=value>
```

`name` 与 JSON 字段路径完全对应，但省略最外层的 `op`、`pta` 或 `tran`。命令行中字段名可使用下划线或连字符，且不区分大小写；例如 JSON 的 `op.newton.maximum_iterations` 可写为 `newton.maximum-iterations=200` 或 `newton.maximum_iterations=200`。

```sh
# 覆盖 OP 的 Newton 与 source stepping 参数
./spice --config config.json \
  --op-option newton.maximum-iterations=200 \
  --op-option source-stepping.enabled=false \
  tests/cases/op/level1_01_resistive_bridge_mesh.cir

# --pta 选择 PTA 模式；其余 pta 字段使用 --pta-option
./spice --pta fallback \
  --pta-option newton.tolerance=10n \
  --pta-option compound-time-constant=10n \
  --pta-option include-diodes=true \
  tests/cases/op/level1_01_resistive_bridge_mesh.cir

# 覆盖或创建 TRAN 分析，并调整瞬态求解器参数
./spice --tran-option output-interval=1n \
  --tran-option stop-time=100n \
  --tran-option solver.maximum-rejects=20 \
  --tran-option solver.newton.maximum-iterations=500 \
  tests/cases/op/level1_01_resistive_bridge_mesh.cir
```

`--op-option` 支持 `newton.*` 与 `source-stepping.*` 的所有字段。`--pta-option` 支持 `pta` 的全部字段，PTA 模式则继续使用现有的 `--pta disabled|force|fallback`；`initial-bjt-vbe=null` 可清空该可选值。`--tran-option` 支持 `tran` 顶层字段与 `solver.*` 的全部字段，`enabled=false` 可禁用瞬态分析，其他 TRAN 字段会在不存在 `.tran` 时创建一个瞬态分析配置；新建配置仍必须最终提供有效的 `output-interval` 与 `stop-time`。所有数值均支持 SPICE 单位后缀（如 `1n`、`10u`），布尔值接受 `true`/`false` 或 `1`/`0`。

覆盖优先级为“内建默认值 < `config.json` < 显式 CLI 分析参数 < 网表控制卡”。配置文件与命令行的同名值相互冲突时仍由命令行优先；但只要 `.cir` / `.sp` 中已显式设置该参数，程序便静默保留网表值，不输出警告或错误。

目前受保护的网表控制字段包括 `.tran` 的 `TSTEP`、`TSTOP`、显式 `TSTART`、`TMAX` 和 `UIC`，`.options DELMAX`，以及 `.pstran` 中显式给出的 `convval`、`initstep`、`minstep`、`maxstep`、`tau`、`vbe0`、`tauramp` 与 PTA 模式。未由网表给出的 OP 参数、TRAN 求解器参数及 PTA 其他参数仍可由配置文件或命令行注入。`.pstran` 始终强制 PTA 模式，因此与 `--pta` 或 `pta.mode` 冲突时会静默保留 `force`。网表的 `TMAX` / `DELMAX` 是硬上限，外部的 `tran.maximum_step` 或 `--tran-option maximum-step=...` 不会改变它。网表含 `.tran` 时，外部 `enabled=false` 也不会禁用该分析；没有 `.tran` 时，`enabled: false` 仍可禁用由外部配置创建的瞬态分析。

查看命令行帮助：

```sh
./spice --help
```

## 自动测试

测试与参考结果按分析类型分离：

```text
tests/
  cases/
    op/          18 个 operating-point netlist
    tran/        18 个 transient netlist
  references/
    op/          18 个由 ngspice 独立生成的 OP listing reference
    tran/        18 个由 ngspice 独立生成并重采样的 TRAN listing reference
  output/
    op/          测试产生的 .out / .raw / .err
    tran/        测试产生的 .out / .raw / .err
```

运行全部 36 个用例：

```sh
make test
```

`make test-config` 可单独运行配置模块单元测试和配置 CLI 端到端测试。

`make test-op` 与 `make test-tran` 会在每个网表执行后输出一条
`TIME <analysis> <case> <milliseconds> PASS/FAIL`，并输出该分析组的总墙钟时间。单例时间覆盖 simulator 子进程启动、解析、建模、求解以及 `.out` / `.raw` 写出；rawfile 校验和 ngspice 对照时间不包含在其中，便于 PTA 前后比较求解端到端开销。

PTA Force OP 回归与参考精度比较可一并运行：

```sh
make pta-force-standard
```

当前该套件覆盖 18 个 OP 网表和 76 个输出值。它是 PTA 的基础回归门槛，不替代更大规模的困难非线性电路基准。

另有一条专门的困难 OP 基准：弱偏置交叉耦合 CMOS 锁存器。普通
Newton（包括自动 source stepping）预期失败；调优后的冷启动 Force PTA 与普通路径
失败后的 Fallback PTA 均应落到与 ngspice 46 独立参考一致的 `q-high` 稳定支路。
该锁存器有多个 DC 解，因此参考值用于验证所选稳定支路，而不宣称解唯一：

```sh
make test-pta-hard-op
```

这是求解器分流基准，不属于默认 `make test` 的永久发布门槛；如果普通 Newton 日后也能
求解该电路，应同步更新此基准，而不是把算法改进当作回归。

也可以分别运行或只比较已有结果：

```sh
make test-io
make test-cases
make test-op
make test-tran
make test-netlists  # 递归解析 tests/ 下所有 .cir / .sp，不执行求解
make test-private   # 仅解析 tests/private/
make compare
make compare-op
make compare-tran
```

`make test` 会完成以下检查：

1. 构建 simulator。
2. 使用 `tests/scripts/test_io.py` 检查 SPICE 注释、续行、大小写、严格数值/model/实例参数、`.end`、混合 OP/TRAN 输出、事务式文件替换、硬链接保护和 CLI。
3. 对每个 netlist 同时生成 listing、ASCII rawfile 和 stderr 文件。
4. 使用 `tests/scripts/validate_raw.py` 校验 rawfile header、变量数量、点数、有限数值和瞬态时间单调性，并把 raw 数据与同次 listing 逐点、逐变量交叉核对。
5. 使用 `tests/scripts/compare_spice.py` 解析标准与实际 `Index` 表格，并按绝对误差加相对误差比较。
6. 使用 `--parse-only` 递归读取 `tests/` 下全部 `.cir` / `.sp`，覆盖 `tests/private/` 的遗留 Level-3 MOS 模型卡与层次化子电路；此检查只验证语法、实例、模型和分析参数的读取与校验，不要求求解收敛。

`make test-cases` 是独立的网表复杂度审计：检查两组各 18 个网表的命名、数量、分析类型、物理行数、有效语句以及最大案例规模；它不属于 `make test` 的常规回归流程。

默认容差：

```make
OP_ABS_TOL    ?= 5e-4
OP_REL_TOL    ?= 1e-4
TRAN_ABS_TOL  ?= 1e-4
TRAN_REL_TOL  ?= 1e-3
TIME_ABS_TOL  ?= 1e-15
```

判定公式：

```text
|actual - expected| <= absolute_tolerance + relative_tolerance * |expected|
```

详细显示每一个比较值：

```sh
make test OP_COMPARE_FLAGS=--verbose TRAN_COMPARE_FLAGS=--verbose
```

OP 与 TRAN 各有 18 个由上游开源电路库拓扑改编的压力用例，Level 分布均为 `4/4/5/5`：Level 1 为 10-20 行，Level 2 为 20-40 行，Level 3 为 40-100 行，Level 4 大于 100 行；两组最大案例均为 340 个物理行。案例不是上游文件的原样复制，而是展平并约束到当前解析器所支持的 primitive-only 子集。详细来源、适配规则和逐级行数见 [`tests/cases/SOURCES.md`](tests/cases/SOURCES.md)。

测试用例与参考值可分别复现：

```sh
python3 -B tests/scripts/generate_complex_cases.py
make test-cases
make generate-standards  # 需要 ngspice
```

`tests/references/` 不使用本项目求解结果自我生成。OP 参考值直接来自 ngspice 46；生成 TRAN 参考时，脚本会向临时网表注入固定的高精度 ngspice 设置：`reltol=1e-8`、`vntol=1e-10`、`abstol=1e-12`、`trtol=1`，并将内部最大步长限制为 `TSTEP / 2000`。随后将 ngspice 结果线性重采样到网表要求的输出时间网格，原始测试网表不会被修改。对于 `UIC` 网表，仅显式 `t=0` 行按本项目当前的全零采样约定处理，所有 `t>0` 数据均来自 ngspice。

## 目录结构

```text
include/
  analysis/      分析计划、瞬态配置、stamp 上下文与积分器
  app/           命令行接口
  circuit/       Circuit 求解编排与 NodeMap 拓扑接口
  config/        配置加载、类型化覆盖及参数优先级接口
  devices/       器件定义及 OP / TRAN stamp
  io/            SPICE listing / rawfile 与事务式文件输出接口
  math/          Eigen 稀疏 MNA、Newton 步长控制与数值限制工具
  models/        .model 参数存储
  netlist/       网表读取、层次展开与 Parser 接口
  utils/         跨模块字符串与 SPICE 数值工具
src/
  app/           命令行解析
  circuit/       Circuit 与 NodeMap 实现
  config/        配置加载、schema 解析和覆盖应用
  io/            listing / rawfile 格式化与文件提交
  netlist/       网表读取、子电路展开与语义解析
  main.cpp       应用入口与高层流程编排
tests/
  cases/         OP / TRAN netlist 与 SOURCES.md
  references/    ngspice 独立参考 listing
  scripts/       用例生成、参考生成与回归校验脚本
  output/        测试生成的 listing / rawfile / stderr（不纳入版本控制）
```

项目内头文件统一相对于 `include/` 引用，文件名统一使用 snake_case。`include/analysis`、`include/devices`、`include/math` 和 `include/models` 当前主要是 header-only 模块；存在独立实现文件的模块则在 `src/` 中使用对应职责目录。Makefile 自动收集 `src/` 及其一级职责目录中的 `.cpp` 文件。默认构建使用 `-O3`；调试时可用 `make OPT_FLAGS=-O0` 覆盖。

## 当前限制

- 不支持 `PULSE`、`SIN`、`PWL` 等时变独立源，因此瞬态阶跃测试使用 `UIC` 和固定 DC 源构造 t=0 激励。
- 瞬态使用首步 Backward Euler 与受步长比限制的可变步长 BDF2；具备基于预测—校正差/step-doubling 的误差控制和步长拒绝重试，但尚未实现严格 LTE 估计、事件断点对齐或高阶积分公式。
- `UIC` 当前把完整 MNA 解向量初始化为零；尚未支持器件 `IC=`、`.ic` 与一致初值求解。
- 瞬态 Newton 失败会缩小时间步并在上一个已接受状态重试；非线性收敛判据本身仍未拆分电压/电流的相对与绝对容差。
- PTA 已具备伪元件 stamp、BE/BDF2 伪时间推进、失败缩步、最小步长后的全局增容，以及成功步后的逐节点振荡降容；导数与 DC 残差判据已经归一化，但默认容差仍需通过更广泛的困难非线性电路基准验证。
- 不支持 `.include`、`.lib`、全局 `.param`、`.temp`、`.nodeset`、`.ic`、`.save`。
- 不支持受控源 `E/F/G/H`、行为源、AC/noise 分析。
- 二极管、BJT 和 MOSFET 是简化模型；`RS`、`RBE`、`RCE`、`RDS` 虽可解析但尚未参与 stamp，且瞬态中没有结电容等器件内部动态。
- 电阻、电容、二极管、BJT、MOSFET 的器件电流尚不能通过 `.print i(...)` 输出。

## SPICE 格式参考

I/O 兼容规则主要参考：

- [ngspice User's Manual](https://ngspice.sourceforge.io/docs/ngspice-manual.pdf)
- [ngspice ASCII rawfile writer source](https://sourceforge.net/p/ngspice/ngspice/ci/master/tree/src/frontend/rawfile.c)

本项目对未实现的语法保持显式报错，并在本文档中标明与 ngspice 的差异。
