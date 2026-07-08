# MechForge — 数学文档

## 目录

1. [约束方程组](#1-约束方程组)
2. [位置分析：Newton-Raphson 迭代](#2-位置分析newton-raphson-迭代)
3. [速度分析：线性方程组求解](#3-速度分析线性方程组求解)
4. [加速度分析](#4-加速度分析)
5. [约束类型详解](#5-约束类型详解)
   - [5.1 FixedConstraint（固定约束）](#51-fixedconstraint固定约束)
   - [5.2 DistanceConstraint（距离约束）](#52-distanceconstraint距离约束)
   - [5.3 PrismaticConstraint（移动副约束）](#53-prismaticconstraint移动副约束)
   - [5.4 PositionDriving（位置驱动）](#54-positiondriving位置驱动)
   - [5.5 AngleDriving（角度驱动）](#55-angledriving角度驱动)
   - [5.6 DistanceDriving（距离驱动）](#56-distancedriving距离驱动)
6. [奇异系统与最小二乘解](#6-奇异系统与最小二乘解)

---

## 1. 约束方程组

机构的运动学分析通过一组约束方程来描述。设广义坐标向量为 $q \in \mathbb{R}^{n}$（每个 Joint 有 $(x, y)$ 两个坐标），则所有约束方程构成一个非线性系统：

$$
\Phi(q, t) = 0
$$

其中 $\Phi: \mathbb{R}^{n} \times \mathbb{R} \to \mathbb{R}^{m}$，$m$ 为所有约束方程的总数.

对时间求一阶导，得到速度层的约束：

$$
\frac{d\Phi}{dt} = \frac{\partial \Phi}{\partial q} \dot{q} + \frac{\partial \Phi}{\partial t} = 0
\quad\Longrightarrow\quad
J\,v = B_v
$$

其中 $J = \partial\Phi/\partial q$ 为 Jacobian 矩阵，$v = \dot{q}$ 为广义速度，$B_v = -\partial\Phi/\partial t$ 为速度右端项.

对时间求二阶导，得到加速度层的约束：

$$
\frac{d^2\Phi}{dt^2} = J\,\ddot{q} + \dot{J}\,\dot{q} + \frac{\partial}{\partial t}\!\left(\frac{\partial\Phi}{\partial t}\right) = 0
\quad\Longrightarrow\quad
J\,a = B_a
$$

其中 $a = \ddot{q}$ 为广义加速度，$B_a = -\dot{J}\,v - \dfrac{\partial}{\partial t}\!\left(\dfrac{\partial\Phi}{\partial t}\right)$.

---

## 2. 位置分析：Newton-Raphson 迭代

给定时间 $t$，求解 $\Phi(q, t) = 0$ 得到广义坐标 $q$.

使用 Newton-Raphson 法迭代求解：

1. 给定初始猜测 $q^{(0)}$
2. 对第 $k$ 步，计算残差 $\Phi(q^{(k)}, t)$ 和 Jacobian $J(q^{(k)}, t)$
3. 求解线性系统：
   $$J\,\Delta q = -\Phi(q^{(k)}, t)$$
4. 更新：
   $$q^{(k+1)} = q^{(k)} + \Delta q$$
5. 若 $\|\Delta q\| < \text{tol}$ 则收敛，否则继续迭代

当 $J$ 列满秩时，直接用 QR 分解求解：
$$\Delta q = \text{solve}(J, -\Phi)$$

当 $J$ 奇异（行数 $\neq$ 列数或秩不足）时，使用 SVD 求最小二乘解：
$$\Delta q = J^{+}(-\Phi)$$
其中 $J^{+}$ 为 Moore-Penrose 伪逆.

> **实现细节**（`solver.cpp` `solvePosition_oneStep`）：
> - 使用 Eigen 的 `colPivHouseholderQr().solve()` 求解 QR 分解
> - 若 `qr.rank() < J.cols()`（秩亏缺），回退到 `jacobiSvd().solve()` 求最小二乘解
> - 收敛后做残差检查，`||Φ|| > tol × 100` 时报冲突约束错误

---

## 3. 速度分析：线性方程组求解

位置 $q$ 已知后，速度 $v$ 由线性方程组直接求解：

$$J\,v = B_v$$

其中 $B_v = -\partial\Phi/\partial t$ 由各约束的 `velRHS` 组装而成.

这是一个线性系统，直接使用 QR 分解求解：
$$v = J^{-1} B_v$$

> **实现细节**（`solver.cpp` `solveVelocity_oneStep`）：
> - 使用 `J.colPivHouseholderQr().solve(Bv)` 直接求解

---

## 4. 加速度分析

位置 $q$ 和速度 $v$ 已知后，加速度 $a$ 由线性方程组求解：

$$J\,a = B_a$$

其中 $B_a$ 的表达式为：
$$B_a = -\dot{J}\,v - \frac{\partial}{\partial t}\!\left(\frac{\partial\Phi}{\partial t}\right)$$

在代码中，$B_a$ 由各约束的 `accRHS` 直接计算给出，避免了显式计算 $\dot{J}$ 的繁琐.

> **实现细节**（`solver.cpp` `solveAcceleration_oneStep`）：
> - 使用 `J.colPivHouseholderQr().solve(Ba)` 直接求解

---

## 5. 约束类型详解

### 5.1 FixedConstraint（固定约束）

将 Joint $i$ 固定在已知位置 $p_0 = (x_0, y_0)$.

**约束方程**（2 个方程）：
$$\Phi = \begin{bmatrix} x_i - x_0 \\ y_i - y_0 \end{bmatrix} = 0$$

**Jacobian**：
$$J = \begin{bmatrix}
\cdots & 1 & 0 & \cdots \\
\cdots & 0 & 1 & \cdots
\end{bmatrix}$$

即在 Joint $i$ 对应的 $(x, y)$ 列处为 $I_{2\times 2}$，其余为 $0$.

**速度/加速度右端项**：
$$B_v = 0,\quad B_a = 0$$

---

### 5.2 DistanceConstraint（距离约束）

Joint $A$ 和 Joint $B$ 之间的距离保持固定值 $d$.

**约束方程**（1 个方程）：
$$\Phi = (x_A - x_B)^2 + (y_A - y_B)^2 - d^2 = 0$$

**Jacobian**（1 行 $n$ 列）：
$$J = \begin{bmatrix}
\cdots & 2\Delta x & 2\Delta y & \cdots & -2\Delta x & -2\Delta y & \cdots
\end{bmatrix}$$

其中 $\Delta x = x_A - x_B$，$\Delta y = y_A - y_B$.

**速度右端项**：
$$B_v = 0$$

**加速度右端项**：
$$B_a = -2\bigl((\dot{x}_A - \dot{x}_B)^2 + (\dot{y}_A - \dot{y}_B)^2\bigr)$$

这是因为：
$$\dot{\Phi} = 2\Delta x\,\Delta\dot{x} + 2\Delta y\,\Delta\dot{y}$$
$$\ddot{\Phi} = 2(\Delta\dot{x}^2 + \Delta\dot{y}^2) + 2\Delta x\,\Delta\ddot{x} + 2\Delta y\,\Delta\ddot{y}$$

由 $\ddot{\Phi} = 0$ 可得 $J\,a = B_a$，其中 $B_a = -2(\Delta\dot{x}^2 + \Delta\dot{y}^2)$.

---

### 5.3 PrismaticConstraint（移动副约束）

Joint $i$ 被约束在一条过 $p_0$、方向为 $d$ 的直线上滑动.

**约束方程**（1 个方程，点 $p$ 到直线 $p_0 + \lambda d$ 的距离为 0）：
$$\Phi = (y_i - y_0)d_x - (x_i - x_0)d_y = 0$$

这相当于向量 $(p - p_0)$ 与方向 $d$ 的叉积（二维中的标量）为 0，即 $p$ 在直线上.

**Jacobian**（1 行 $n$ 列）：
$$J = \begin{bmatrix}
\cdots & -d_y & d_x & \cdots
\end{bmatrix}$$

其中系数来自 $\partial\Phi/\partial x_i = -d_y$，$\partial\Phi/\partial y_i = d_x$.

**速度/加速度右端项**：
$$B_v = 0,\quad B_a = 0$$

---

### 5.4 PositionDriving（位置驱动）

Joint $j$ 相对于 Joint $r$ 的位置由时间函数 $f(t): \mathbb{R} \to \mathbb{R}^2$ 驱动.

**约束方程**（2 个方程）：
$$\Phi = \begin{bmatrix}
x_j - x_r - f_x(t) \\
y_j - y_r - f_y(t)
\end{bmatrix} = 0$$

**Jacobian**（2 行 $n$ 列）：
$$J = \begin{bmatrix}
\cdots & 1 & 0 & \cdots & -1 & 0 & \cdots \\
\cdots & 0 & 1 & \cdots & 0 & -1 & \cdots
\end{bmatrix}$$

即在 Joint $j$ 处为 $I_{2\times 2}$，Joint $r$ 处为 $-I_{2\times 2}$.

**速度右端项**：
$$B_v = \dot{f}(t)$$

**加速度右端项**：
$$B_a = \ddot{f}(t)$$

---

### 5.5 AngleDriving（角度驱动）

Joint $j$ 相对于 Joint $r$ 的连线角度由时间函数 $\theta(t)$ 驱动.

设向量 $p_j - p_r = (\Delta x, \Delta y)$，约束方程为：
$$\Phi = \Delta x \sin\theta(t) - \Delta y \cos\theta(t) = 0$$

这个方程的几何意义是：向量 $(\Delta x, \Delta y)$ 与 $(\cos\theta, \sin\theta)$ 方向平行，即连线角度为 $\theta$.

**Jacobian**（1 行 $n$ 列）：
$$
J = \begin{bmatrix}
\cdots & \sin\theta & -\cos\theta & \cdots & -\sin\theta & \cos\theta & \cdots
\end{bmatrix}
$$

其中 Joint $j$ 处为 $(\sin\theta, -\cos\theta)$，Joint $r$ 处为 $(-\sin\theta, \cos\theta)$.

**速度右端项**：
$$B_v = -(\Delta x\cos\theta + \Delta y\sin\theta)\,\dot{\theta}$$

推导：
$$\frac{\partial\Phi}{\partial t} = \Delta x\cos\theta\cdot\dot{\theta} + \Delta y\sin\theta\cdot\dot{\theta} = (\Delta x\cos\theta + \Delta y\sin\theta)\dot{\theta}$$
$$B_v = -\frac{\partial\Phi}{\partial t} = -(\Delta x\cos\theta + \Delta y\sin\theta)\dot{\theta}$$

**加速度右端项**：
$$
\begin{aligned}
B_a = &-\alpha(\Delta x\cos\theta + \Delta y\sin\theta) \\
      &-2\omega(\Delta\dot{x}\cos\theta + \Delta\dot{y}\sin\theta) \\
      &+\omega^2(\Delta x\sin\theta - \Delta y\cos\theta)
\end{aligned}
$$

其中 $\omega = \dot{\theta}$，$\alpha = \ddot{\theta}$，$\Delta\dot{x} = \dot{x}_j - \dot{x}_r$，$\Delta\dot{y} = \dot{y}_j - \dot{y}_r$.

---

### 5.6 DistanceDriving（距离驱动）

Joint $A$ 和 Joint $B$ 之间的距离由时间函数 $L(t)$ 驱动.

**约束方程**（1 个方程）：
$$\Phi = (x_A - x_B)^2 + (y_A - y_B)^2 - L(t)^2 = 0$$

**Jacobian**（同 5.2 DistanceConstraint）：
$$J = \begin{bmatrix}
\cdots & 2\Delta x & 2\Delta y & \cdots & -2\Delta x & -2\Delta y & \cdots
\end{bmatrix}$$

**速度右端项**：
$$B_v = 2L(t)\,\dot{L}(t)$$

**加速度右端项**：
$$B_a = 2\bigl(\dot{L}^2 + L\ddot{L}\bigr) - 2\bigl(\Delta\dot{x}^2 + \Delta\dot{y}^2\bigr)$$

---

## 6. 奇异系统与最小二乘解

当约束方程的个数不等于广义坐标数时，Jacobian $J$ 不是方阵，需要用最小二乘法求解.

### 超定系统（$m > n$）

方程数多于变量数，求最小二乘解：
$$\min_{\Delta q} \|J\Delta q + \Phi\|^2$$

使用 QR 分解求解：
$$J = QR \quad\Longrightarrow\quad \Delta q = R^{-1}Q^\top(-\Phi)$$

### 欠定系统（$m < n$）

变量数多于方程数，系统有无穷多解，取最小范数解：
$$\min_{\Delta q} \|\Delta q\|^2 \quad \text{s.t.} \quad J\Delta q = -\Phi$$

使用 SVD 求解：
$$J = U\Sigma V^\top \quad\Longrightarrow\quad \Delta q = V\Sigma^{+}U^\top(-\Phi)$$

其中 $\Sigma^{+}$ 是 $\Sigma$ 的伪逆（非零奇异值的倒数）.

### 代码中的处理逻辑

```cpp
auto qr = J.colPivHouseholderQr();

if (qr.rank() < J.cols()) {
    // 秩亏缺 → 使用 SVD 求最小范数解
    auto svd = J.jacobiSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>();
    dq = svd.solve(-F);
} else {
    // 满秩 → QR 最小二乘解
    dq = qr.solve(-F);
}
```
