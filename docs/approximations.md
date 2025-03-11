## Approximation Strategies
The following sections suggest recursive equations for various discrete approximation strategies. Each recursion is performed on a discretized time grid with $N$ number of nodes, with $i = 1, 2, \ldots, N$. The implementation details can be found in the corresponding example code.

### Simple Rounding

For the following approximation of $v(t)$:
$$
\min_{v(t)} \|v(t) - \tilde{v}(t)\| \qquad \forall t \in [t_0, t_f],
$$
we apply the following recursion:
$$
\begin{aligned}
c_i(v_i) &= \|v_i - \tilde{v}_i\|, \\
\pi_i(v_i) &= \operatorname{argmin}_{v_{i-1}} \{ V_{i-1}(v_{i-1}) + c_i(v_i)\}, \\
V_i(v_i) &= V_{i-1}(\pi_i(v_i)) + c_i(v_i), \\
V_0(v_0) &= c_0(v_0).
\end{aligned}
$$
**Examples:** `mwe`

### SumUp Rounding ($n_v = 1$)

For the following approximation of $v(t)$:
$$
\min_{v(t)} \bigg|\int_{t_0}^{t_f} v(t) - \tilde{v}(t)~dt \bigg|
$$
we use the recursion:
$$
\begin{aligned}
c_i(v_i) &= v_i - \tilde{v}_i, \\
\pi_i(v_i) &= \operatorname{argmin}_{v_{i-1}} \{\big| V_{i-1}(v_{i-1}) + c_i(v_i)\big|\}, \\
V_i(v_i) &= V_{i-1}(\pi_i(v_i)) + c_i(v_i), \\
V_0(v_0) &= c_0(v_0).
\end{aligned}
$$
**Examples:** `rck`, `trj`, `lvf`

### SumUp Rounding ($n_v > 1$)

If input dimensions are uncoupled, each dimension can be treated separately as in the $n_v = 1$ case. If they are coupled, for instance via the SOS1 condition $\mathbf{1}^\top v(t) = 1$, we can use the infinity norm:
$$
\min_{v(t)} \bigg\|\int_{t_0}^{t_f} v(t) - \tilde{v}(t)~dt \bigg\|_{\infty}
$$
This is achieved through:
$$
\begin{aligned}
c_i(v_i) &= v_i - \tilde{v}_i, \\
\pi_i(v_i) &= \operatorname{argmin}_{v_{i-1}} \{\big\| V_{i-1}(v_{i-1}) + c_i(v_i)\big\|_{\infty}\}, \\
V_i(v_i) &= V_{i-1}(\pi_i(v_i)) + c_i(v_i),\\
V_0(v_0) &= c_0(v_0).
\end{aligned}
$$
**Examples:** `sos`

### CIA Rounding
For the following approximation of $v(t)$:

$$
\min_{v(t)} \max_{t \in [t_0, t_f]} \bigg\|\int_{t_0}^{t} v(\tau) - \tilde{v}(\tau)~d\tau \bigg\|_\infty
$$
we use the recursion:
$$
\begin{aligned}
c_i(v_i) &= v_i - \tilde{v}_i, \\
\pi_i(v_i) &= \operatorname{argmin}_{v_{i-1}} \max \{\big\| A V_{i-1}(v_{i-1}) + c_i(v_i)\big\|_{\infty}, b V_{i-1}(v_{i-1})\},\ \\
V_i(v_i) &= \begin{bmatrix}
A V_{i-1}(\pi_i(v_i)) + c_i(v_i)\\
\max \{\big\| A V_{i-1}(\pi_i(v_i)) + c_i(v_i)\big\|_{\infty}, b V_{i-1}(\pi_i(v_i))\}\
\end{bmatrix}, \\
V_0(v_0) &= \begin{bmatrix} c(v_0)\\ \big\| c(v_0)\big\|_{\infty} \end{bmatrix},
\end{aligned}
$$
where $A$ consists of the first $n_v - 1$ rows of the identity matrix $I_{n_v \times n_v}$, and $b$ is the last row of $I_{n_v \times n_v}$.

**Examples:** `mat`
