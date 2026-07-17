# MechForge CLI — JSON 通信协议

`mechforge_solve` 从 stdin 读取 JSON，求解后输出 JSON 到 stdout。

---

## 输入 JSON 格式

顶层对象：

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `joints` | `Joint[]` | 是 | 关节列表 |
| `links` | `Link[]` | 否 | 杆件列表 |
| `drivings` | `Driving[]` | 否 | 驱动约束列表（仅允许写入驱动约束，静态约束由程序自动推导） |
| `solver` | `SolverConfig` | 否 | 求解器配置 |

---

### `Joint`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 关节编号，从 0 开始，不可重复 |
| `type` | integer | 是 | `0` 表示 `Grounded`, `1` 表示 `Revolute`, `2` 表示 `Prismatic`, `3` 表示 `Free`, 其余表示 `Free` |
| `groundPos` | `[number, number]` | 否 | 当 `type` 为 `0` 时，必须提供固定点坐标 `[x, y]`，其余情况无需提供 |
| `slide` | `Slide` | 否 | 当 `type` 为 `2` 时，必须提供滑动约束参数，其余情况无需提供 |
| `pos` | `[number, number]` | 是 | 初始位置 `[x, y]` |
| `vel` | `[number, number]` | 否 | 初始速度 `[vx, vy]` |
| `acc` | `[number, number]` | 否 | 初始加速度 `[ax, ay]` |

#### `Slide`：

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `origin` | `[number, number]` | 是 | 直线上一点 `[x, y]` |
| `direction` | `[number, number]` | 是 | 方向向量 `[dx, dy]`，单位向量更推荐，但不强制 |



### `Link`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 杆件编号，不可重复 |
| `jointA` | integer | 是 | 一端关节的 `id` |
| `jointB` | integer | 是 | 另一端关节的 `id` |
| `length` | number | 是 | 杆长，大于 0 |

### `Driving`

`kind` 区分驱动类型：

#### `kind: "position_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `type` | integer | 是 | `0` 表示 `Position` |
| `jointId` | integer | 是 | 被驱动的关节 `id` |
| `relativeId` | integer | 是 | 参考关节 `id` |
| `posX` | string | 是 | `x(t)` 表达式 |
| `posY` | string | 是 | `y(t)` 表达式 |
| `velX` | string | 是 | `ẋ(t)` 表达式 |
| `velY` | string | 是 | `ẏ(t)` 表达式 |
| `accX` | string | 是 | `ẍ(t)` 表达式 |
| `accY` | string | 是 | `ÿ(t)` 表达式 |

#### `kind: "angle_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `type` | integer | 是 | `1` 表示 `Angle` |
| `jointId` | integer | 是 | 被驱动的关节 `id` |
| `relativeId` | integer | 是 | 参考关节 `id` |
| `theta` | string | 是 | `θ(t)` 表达式 (rad) |
| `omega` | string | 是 | `ω(t)` 表达式 (rad/s) |
| `alpha` | string | 是 | `α(t)` 表达式 (rad/s²) |

#### `kind: "distance_driving"`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `type` | integer | 是 | `2` 表示 `Distance` |
| `jointA` | integer | 是 | 一端关节 `id` |
| `jointB` | integer | 是 | 另一端关节 `id` |
| `distance` | string | 是 | `L(t)` 表达式 |
| `vel` | string | 是 | `L̇(t)` 表达式 |
| `acc` | string | 是 | `L̈(t)` 表达式 |

---

### `SolverConfig`

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `endTime` | number | 否 | `-1` | 仿真终止时间. 小于等于 `0` 表示直到无穷; 大于 `0`, 且大于等于 `k * step`, 但小于 `(k+1) * step` 表示求解 `k` 步 |
| `step` | number | 否 | `0.01` | 时间步长 |
| `solveLevel` | number | 否 | `0` | `0` 表示位置，`1` 表示速度，`2` 表示加速度 |
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
| `joints` | `JointResult[]` | 仅成功时 | 各关节位置、速度和加速度 |
| `message` | string | 仅错误时 | 错误描述 |

### `JointResult`

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `id` | integer | 是 | 关节编号 |
| `x` | number | 是 | x 坐标 |
| `y` | number | 是 | y 坐标 |
| `vx` | number | 否 | x 方向速度 |
| `vy` | number | 否 | y 方向速度 |
| `ax` | number | 否 | x 方向加速度 |
| `ay` | number | 否 | y 方向加速度 |