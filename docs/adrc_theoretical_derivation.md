# Mathematical Formulation & Theoretical Derivation of LADRC

This document provides the theoretical derivation of the **Linear Active Disturbance Rejection Control (LADRC)** framework implemented in this project for the active vibration suppression of a compliant link mechanism.

---

## 1. Flexible Beam Dynamics (Euler-Bernoulli Formulation)

A slender flexible cantilever beam of length $L$, mass density $\rho$, cross-sectional area $A$, Young's modulus $E$, and area moment of inertia $I$ subject to transverse displacement $w(x,t)$ is governed by the fourth-order partial differential equation (PDE):

$$\rho A \frac{\partial^2 w(x,t)}{\partial t^2} + E I \frac{\partial^4 w(x,t)}{\partial x^4} = F_{\text{ext}}(x,t)$$

When the root actuator rotates with angular acceleration $\ddot{\theta}(t)$, an inertial D'Alembert body force is distributed along the link:

$$F_{\text{inertial}}(x,t) = -\rho A x \ddot{\theta}(t)$$

Applying modal separation of variables for the dominant fundamental bending mode:

$$w(x,t) = \phi_1(x) \eta_1(t)$$

Evaluating dynamic acceleration at the beam tip ($x = L$), the tip transverse vibration $y(t) = \ddot{w}(L,t)$ collapses into an underdamped second-order ordinary differential equation:

$$m \ddot{y}(t) + c \dot{y}(t) + k y(t) = b_{\text{in}} u(t) + d_{\text{ext}}(t)$$

where:
* $m$: Generalized modal mass
* $c$: Structural and aerodynamic damping coefficient
* $k$: Equivalent bending stiffness
* $u(t)$: Actuator motor voltage / PWM duty cycle
* $b_{\text{in}}$: Actuator torque-to-acceleration gain
* $d_{\text{ext}}(t)$: Unmodeled external disturbances (gear backlash, aerodynamic drag)

---

## 2. ADRC Acceleration-Domain Transformation

Dividing by modal mass $m$:

$$\ddot{y}(t) = -\frac{c}{m}\dot{y}(t) - \frac{k}{m}y(t) + \frac{d_{\text{ext}}(t)}{m} + \frac{b_{\text{in}}}{m}u(t)$$

In classical control, derivative terms require differentiation of sensor signals, which dramatically amplifies high-frequency noise. In ADRC, the plant is mapped to a canonical first-order integrator chain in the acceleration manifold:

$$\dot{y}(t) = f + b_0 u(t)$$

where:
* $b_0 \approx \frac{b_{\text{in}}}{m}$: Known nominal input gain estimate
* $f$: **Total Disturbance**, defined as:

$$f(y, \dot{y}, d_{\text{ext}}, t) = -\frac{c}{m}\dot{y}(t) - \frac{k}{m}y(t) + \frac{d_{\text{ext}}(t)}{m} + (b - b_0)u(t)$$

The Total Disturbance $f$ unifies internal flexural strain forces, structural damping, higher-order modal resonance, air resistance, and motor deadband into a single generalized state.

---

## 3. Second-Order Linear Extended State Observer (LESO)

Treating $f$ as an augmented state $x_2 = f$ with unknown derivative $h = \dot{f}$, the state-space system is:

$$\begin{aligned}
\dot{x}_1 &= x_2 + b_0 u \\
\dot{x}_2 &= h \\
y &= x_1
\end{aligned}$$

The continuous LESO estimates states $z_1 \rightarrow y$ and $z_2 \rightarrow f$:

$$\begin{aligned}
\dot{z}_1 &= z_2 - \beta_1(z_1 - y) + b_0 u \\
\dot{z}_2 &= -\beta_2(z_1 - y)
\end{aligned}$$

Using **Gao's Bandwidth-Parameterization Method**, the observer characteristic equation is placed at a single bandwidth $-\omega_o$:

$$(s + \omega_o)^2 = s^2 + 2\omega_o s + \omega_o^2 = s^2 + \beta_1 s + \beta_2$$

Deterministic observer gains:

$$\beta_1 = 2\omega_o, \quad \beta_2 = \omega_o^2$$

---

## 4. Disturbance Cancellation & Closed-Loop Regulation

With $z_2(t) \approx f(t)$ estimated in real time, the virtual control law cancels the total disturbance:

$$u(t) = \frac{u_0(t) - z_2(t)}{b_0}$$

Substituting into the plant dynamics:

$$\dot{y}(t) = f + b_0 \left(\frac{u_0(t) - z_2(t)}{b_0}\right) \approx u_0(t)$$

The physical non-linear, flexible beam plant is linearized and decoupled into a pure integrator system. For vibration suppression to zero setpoint ($r_{\text{vib}} = 0$):

$$u_0(t) = -k_p z_1(t) \quad \text{with } k_p = \omega_c$$

Overall control law:

$$u_{\text{ADRC}}(t) = \frac{-\omega_c z_1(t) - z_2(t)}{b_0}$$

---

## 5. Discrete Implementation on ESP32 (Forward Euler)

At sample interval $T_s = 0.005\text{ s}$ ($200\text{ Hz}$):

1. **Innovation Error**:
   $$e[k] = z_1[k] - y[k]$$

2. **Observer State Updates**:
   $$z_1[k+1] = z_1[k] + T_s \left(z_2[k] - \beta_1 e[k]\right)$$
   $$z_2[k+1] = z_2[k] + T_s \left(-\beta_2 e[k]\right)$$

3. **Active Control Law Output**:
   $$u_{\text{ADRC}}[k] = \text{constrain}\left(-k_{\text{vib}} z_1[k] - k_{\text{dist}} z_2[k], -u_{\text{max}}, u_{\text{max}}\right)$$

4. **Superposition onto Slew Command**:
   $$u_{\text{motor}}[k] = \text{constrain}\left(U_{\text{target}} + u_{\text{ADRC}}[k], u_{\text{min}}, 255\right)$$
