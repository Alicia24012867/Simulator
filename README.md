# DC Operating Point SPICE-like Simulator

这是一个类 SPICE 的电路仿真器原型，目前目标明确限定为 **DC operating point** 求解。它可以解析一部分 SPICE 风格 netlist，构建 MNA 方程，并用 Newton 迭代求解节点电压和部分支路电流。

当前项目不是完整 SPICE 兼容实现：暂不支持瞬态分析、交流小信号分析、子电路、参数表达式、复杂模型库等功能。

## 当前完成状态

已完成的主流程：

1. 读取 netlist 文件。
2. 解析器件和 `.model`。
3. 建立节点编号表。
4. 为电压源、电感等器件分配额外 branch unknown。
5. 构建稀疏 MNA 矩阵结构。
6. 对线性和非线性器件进行 stamp。
7. 使用 Eigen SparseLU 解线性方程。
8. 对非线性器件执行 Newton 迭代。
9. 输出 DC operating point 结果。
10. 使用测试脚本和参考输出做自动数值判定。

## 支持的器件

目前解析器支持以下 SPICE 元件前缀：

| 前缀 | 器件 | DC 处理 |
| --- | --- | --- |
| `R` | Resistor | 标准电导 stamp |
| `C` | Capacitor | DC 下开路，不贡献矩阵 |
| `L` | Inductor | DC 下短路，等价为 0V 电压源并引入支路电流未知量 |
| `V` | Independent voltage source | MNA branch stamp |
| `I` | Independent current source | RHS stamp |
| `D` | Diode | 指数 I-V 模型，Newton 线性化 |
| `Q` | BJT | 简化 NPN/PNP DC 模型 |
| `M` | MOSFET | 简化 Level-1 风格 NMOS/PMOS 平方律模型 |

## 支持的模型参数

`.model` 当前支持以下类型：

| 类型 | 说明 |
| --- | --- |
| `D` | Diode |
| `NPN` / `PNP` | BJT |
| `NMOS` / `PMOS` / `NCH` / `PCH` | MOSFET |

已读取并用于 DC 计算的主要参数包括：

- Diode: `IS`, `N`, `VT`, `GMIN`
- BJT: `IS`, `BF`, `BR`, `NF`, `NR`, `VT`, `GMIN`
- MOSFET: `VTO` / `VT0`, `KP` / `K`, `LAMBDA` / `LAM`, `GMIN`
- 器件实例参数：`AREA`, `W`, `L`

部分参数虽然会被读取或缓存，但尚未完整实现真实 SPICE 语义，例如 diode `RS` 的完整串联电阻处理。

## Netlist 支持范围

当前支持：

- 第一行作为 title 跳过。
- 空行跳过。
- `*` 开头的注释行。
- `;` 和 `#` 行内注释。
- `+` 续行。
- `.model`。
- `.end`。
- SPICE 数值后缀：`f`, `p`, `n`, `u`, `m`, `k`, `meg`, `g`, `t`, `mil`。
- 电源值写法：`DC 5`, `DC=5`, 或直接写数值。

当前会忽略除 `.model` 和 `.end` 以外的大多数点命令。例如：

- `.op` 目前不作为真正控制命令解析；程序默认执行 operating point。
- `.print op ...` 目前不控制输出项；程序会输出所有节点电压和有 branch unknown 的器件电流。

## 求解方法

核心求解流程在 `Circuit::solve()` 中完成：

- 最大 Newton 迭代次数：`150`
- 收敛阈值：`1.0e-9`
- 最大步长限制：`1.0`
- 收敛判据：相邻两轮解向量的 infinity norm 变化量小于阈值

输出中的 Newton 信息含义：

| 字段 | 含义 |
| --- | --- |
| `converged` | 是否在最大迭代次数内收敛 |
| `iterations` | 实际 Newton 迭代次数 |
| `max_iterations` | 最大允许迭代次数 |
| `final_delta` | 最后一轮解向量最大变化量 |
| `tolerance` | 收敛阈值 |
| `damped_steps` | 被步长限制处理的迭代次数 |
| `cpu_time_seconds` | 求解 CPU 时间 |

## 输出格式

程序输出为自定义精简格式，不是 ngspice 原始报告格式。

示例：

```text
Operating Point
Newton Info
converged yes
iterations 11
max_iterations 150
final_delta 0.0000000000e+00
tolerance 1.0000000000e-09
damped_steps 5
cpu_time_seconds 4.8000000000e-04
Node Voltages
v(vdd) 5.0000000000e+00
v(out) 9.7020526365e-01
Branch Currents
i(VDD) -4.0297947363e-04
```

目前 branch current 只会输出有额外 branch unknown 的器件，例如电压源和电感。电阻、二极管、MOSFET、BJT 的器件电流还没有统一输出。

## 构建依赖

需要：

- C++17 编译器
- Eigen3
- Python 3，用于测试结果比较
- Make

macOS 可通过 Homebrew 安装 Eigen：

```sh
brew install eigen
```

也可以手动指定 Eigen 头文件目录：

```sh
make spice EIGEN_INCLUDE=/path/to/eigen3
```

## 构建和运行

注意：当前 Makefile 的默认目标主要做 Eigen 检查。建议显式运行 `make spice`。

构建：

```sh
make spice
```

运行：

```sh
./spice testcase/level1_01_resistor_divider.cir
```

输出到文件：

```sh
./spice testcase/level1_01_resistor_divider.cir result.out
```

清理构建产物和测试输出：

```sh
make clean
```

## 自动测试

运行全部测试：

```sh
make test
```

`make test` 会：

1. 构建 `spice`。
2. 运行 `testcase/*.cir`。
3. 为每个测试用例创建 `actual/<case>/`。
4. 将输出写入 `actual/<case>/<case>.out` 和 `actual/<case>/<case>.err`。
5. 调用 `scripts/compare_op.py`，从 `standard/*.out` 和 `actual/<case>/<case>.out` 中提取 operating point 数值并比较。

默认比较容差来自 Makefile：

```make
OP_ABS_TOL ?= 1e-3
OP_REL_TOL ?= 2e-3
```

判定公式：

```text
|actual - expected| <= OP_ABS_TOL + OP_REL_TOL * |expected|
```

单独比较已有 `actual/`：

```sh
make compare
```

查看每个比较项的详细 expected、actual、diff 和允许区间：

```sh
make test OP_COMPARE_FLAGS=--verbose
```

覆盖测试容差：

```sh
make test OP_ABS_TOL=1e-4 OP_REL_TOL=1e-3
```

当前测试集包括：

- 线性电阻分压
- 电流源与电阻
- 二极管与电阻负载
- 电感 DC 短路
- NMOS 电阻负载
- NPN 共射电路
- 双电源线性桥式电路
- 二极管与 NMOS 组合负载
- CMOS inverter DC 点
- Diode + NMOS clamp network
- Coupled CMOS inverters with diode clamps
- BJT + diode bias network
- MOS + BJT hybrid load
- NPN + NMOS + diode mixed ring

## 目录结构

```text
include/
  core/          Circuit, Parser, NodeMap 声明
  devices/       各类器件模型和 stamp 实现
  math/          MNA 稀疏矩阵封装
  models/        .model 参数存储和 DC 参数缓存
src/
  core/          Circuit, Parser, NodeMap 实现
testcase/        输入 netlist 测试用例
standard/        参考输出
scripts/         测试比较脚本
utils/           字符串和 SPICE 数值解析工具
```

## 当前限制和待办

主要限制：

- `.op` 和 `.print` 还没有完整语义。
- 输出格式不是 ngspice 兼容格式。
- 非线性模型是简化 DC 模型，与真实 SPICE 模型仍有差距。
- 已实现 PN 结 COLON current-basis limiting、MOSFET 电压限制和动态 source stepping；尚未实现 gmin stepping、pseudo-transient 等高级策略。
- 浮空节点、奇异矩阵、冲突电压源等错误诊断还不够友好。
- 不支持 `.include`, `.lib`, `.param`, `.options`, `.temp`, `.nodeset`, `.ic`, `.subckt`。
- 不支持受控源 `E/F/G/H` 和行为源。
- 器件级电流、功耗和模型详细报告尚未完整输出。

后续优先级建议：

1. 完善 `.op` / `.print op` 的语义。
2. 增加更细的错误诊断和 netlist 报错位置。
3. 改进非线性器件模型和收敛算法。
4. 增加器件电流和功耗输出。
5. 扩充测试覆盖，并把测试结果纳入持续检查流程。
