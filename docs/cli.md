# MechForge CLI — JSON 通信协议

`mechforge_solve` 从 stdin 读取 JSON，求解后输出 JSON 到 stdout。

---

## 输入 JSON 格式

顶层对象：

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `joints` | `Joint[]` | 是 | 关节列表 |
| `links` | `Link[]` | 否 | 杆件列表 |
| `constraints` | `Constraint[]` | 是 | 约束列表（含驱动约束） |
| `solver` | `SolverConfig` | 否 | 求解器配置 |

---

### `Joint`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 关节编号，从 0 开始，不可重复 |
| `pos` | `[number, number]` | 是 | 初始位置 `[x, y]` |

### `Link`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 杆件编号，不可重复 |
| `jointA` | integer | 是 | 一端关节的 `id` |
| `jointB` | integer | 是 | 另一端关节的 `id` |
| `length` | number | 是 | 杆长，大于 0 |

### `Constraint`

`kind` 区分约束类型：

#### `kind: "fixed"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointId` | integer | 是 | 被固定的关节 `id` |
| `pos` | `[number, number]` | 是 | 固定位置 `[x, y]` |

#### `kind: "distance"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointA` | integer | 是 | 一端关节 `id` |
| `jointB` | integer | 是 | 另一端关节 `id` |
| `length` | number | 是 | 距离，大于 0 |

#### `kind: "prismatic"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointId` | integer | 是 | 被约束的关节 `id` |
| `origin` | `[number, number]` | 是 | 直线上一点 `[x, y]` |
| `dir` | `[number, number]` | 是 | 方向向量 `[dx, dy]` |

#### `kind: "position_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointId` | integer | 是 | 被驱动的关节 `id` |
| `relativeTo` | integer | 是 | 参考关节 `id` |
| `posX` | string | 是 | `x(t)` 表达式 |
| `posY` | string | 是 | `y(t)` 表达式 |
| `velX` | string | 是 | `ẋ(t)` 表达式 |
| `velY` | string | 是 | `ẏ(t)` 表达式 |
| `accX` | string | 是 | `ẍ(t)` 表达式 |
| `accY` | string | 是 | `ÿ(t)` 表达式 |

#### `kind: "angle_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointId` | integer | 是 | 被驱动的关节 `id` |
| `relativeTo` | integer | 是 | 参考关节 `id` |
| `theta` | string | 是 | `θ(t)` 表达式 (rad) |
| `omega` | string | 是 | `ω(t)` 表达式 (rad/s) |
| `alpha` | string | 是 | `α(t)` 表达式 (rad/s²) |

#### `kind: "distance_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `jointA` | integer | 是 | 一端关节 `id` |
| `jointB` | integer | 是 | 另一端关节 `id` |
| `distance` | string | 是 | `L(t)` 表达式 |
| `vel` | string | 是 | `L̇(t)` 表达式 |
| `acc` | string | 是 | `L̈(t)` 表达式 |

---

### `SolverConfig`

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `endTime` | number | 否 | `6.28` | 仿真终止时间 |
| `step` | number | 否 | `0.01` | 时间步长 |
| `maxIter` | integer | 否 | `100` | Newton-Raphson 最大迭代次数 |
| `tol` | number | 否 | `1e-9` | NR 收敛容差 |

---

### 表达式语法

变量 `t` 表示时间。支持的运算：`+ - * / ^ %`，函数：`sin cos tan asin acos atan atan2 abs sqrt ceil floor ln log exp`，常量：`pi e`。

---

## 输出 JSON 格式

顶层对象：

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `status` | string | 是 | `"ok"` 或 `"error"` |
| `joints` | `JointResult[]` | 仅成功时 | 各关节最终位置 |
| `message` | string | 仅错误时 | 错误描述 |

### `JointResult`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 关节编号 |
| `x` | number | 是 | 最终 x 坐标 |
| `y` | number | 是 | 最终 y 坐标 |
