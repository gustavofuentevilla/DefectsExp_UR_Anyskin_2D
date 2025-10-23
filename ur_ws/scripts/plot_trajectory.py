#!/usr/bin/env python3
"""
plot_trajectory.py

Standalone helper to sample and plot the `rest_to_rest_trajectory` output X
from your `ur_motion_routines` implementation. This script reproduces the
trajectory calculation (so it doesn't depend on ROS) and plots the 7 values
(x,y,z,qx,qy,qz,qw) over time.

Usage example:
  python3 scripts/plot_trajectory.py

You can pass custom initial/final poses and duration:
  python3 scripts/plot_trajectory.py \
    --pi 0.1 0.0 0.2 0 0 0 1 \
    --pf 0.0 0.0 0.05 -0.7071 0.7071 0.0 0.0 \
    --time 30.0

Dependencies:
  pip install numpy matplotlib

"""

import numpy as np
import matplotlib.pyplot as plt
from math import factorial

def rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t, n):
    """
    Calcula una trayectoria rest-to-rest entre dos poses (P_i y P_f).

    Parámetros:
    -----------
    P_i : array_like
        Pose inicial (vector).
    P_f : array_like
        Pose final (vector).
    t_i : float
        Tiempo inicial del movimiento.
    mvr_time : float
        Duración total del movimiento.
    t : float
        Tiempo actual.
    n : int
        Número de derivadas igualadas a cero en los extremos.

    Retorna:
    --------
    X : ndarray
        Pose interpolada en el tiempo t.
    """

    if n < 1:
        raise ValueError("El orden de derivadas debe ser mayor o igual a 1")

    P_i = np.array(P_i, dtype=float)
    P_f = np.array(P_f, dtype=float)
    P_dim = len(P_i)

    t_f = t_i + mvr_time  # tiempo final

    if t < t_i:
        return P_i
    elif t > t_f:
        return P_f
    else:
        A = np.zeros((n+1, n+1, P_dim))
        Alpha = np.zeros((n+1, 1, P_dim))
        Lambda = np.zeros((n+1, 1, P_dim))
        Lambda[0, 0, :] = 1  # equivalente a MATLAB: Lambda(1,1,:) = 1

        f = np.zeros(P_dim)

        for r in range(P_dim):
            # Construcción de la matriz A
            for i in range(1, n+2):
                for j in range(1, n+2):
                    A[i-1, j-1, r] = ((-1)**(j-1)) * factorial(n + j) / (
                        ((t_f - t_i)**(i-1)) * factorial(n + j - i + 1)
                    )

            # Resolución del sistema lineal
            Alpha[:, 0, r] = np.linalg.solve(A[:, :, r], Lambda[:, 0, r])

            # Cálculo del polinomio f(r)
            tau = (t - t_i) / (t_f - t_i)
            for j in range(0, n+1):
                f[r] += ((-1)**j) * Alpha[j, 0, r] * tau**(n+1+j)

        X = P_i + (P_f - P_i) * f
        return X


def rest_to_rest_trajectory2(P_i, P_f, t_i, mvr_time, t, n):
    """
    Correct rest-to-rest interpolation.

    Construct the minimal-degree polynomial p(tau) with tau in [0,1], degree m=2n+1,
    that satisfies p(0)=0 and derivatives up to order n vanish at tau=0,
    and p(1)=1 with derivatives up to order n vanish at tau=1.

    The trajectory is X = P_i + (P_f - P_i) * p(tau).
    """
    if n < 1:
        raise ValueError("El orden de derivadas debe ser mayor o igual a 1")

    P_i = np.array(P_i, dtype=float)
    P_f = np.array(P_f, dtype=float)

    t_f = t_i + mvr_time

    # Boundary handling
    if t < t_i:
        return P_i
    if t > t_f:
        return P_f

    tau = (t - t_i) / (t_f - t_i)

    # degree and unknown coefficients
    m = 2 * n + 1
    unknown_count = n + 1  # coefficients c_{n+1}..c_m

    # Build linear system at tau=1 for derivatives r=0..n
    M = np.zeros((unknown_count, unknown_count), dtype=float)
    b = np.zeros((unknown_count,), dtype=float)
    b[0] = 1.0

    for r in range(unknown_count):
        for j in range(unknown_count):
            k = n + 1 + j
            if k < r:
                M[r, j] = 0.0
            else:
                # falling factorial k*(k-1)*...*(k-r+1) = k!/(k-r)!
                M[r, j] = float(factorial(k) // factorial(k - r))

    try:
        coeffs_unknown = np.linalg.solve(M, b)
    except np.linalg.LinAlgError:
        coeffs_unknown, *_ = np.linalg.lstsq(M, b, rcond=None)

    coeffs = np.zeros((m + 1,), dtype=float)
    for j in range(unknown_count):
        k = n + 1 + j
        coeffs[k] = coeffs_unknown[j]

    powers = np.array([tau ** k for k in range(m + 1)], dtype=float)
    p = float(np.dot(coeffs, powers))

    X = P_i + (P_f - P_i) * p
    return X


def main():
    # Ejemplo de uso (Prueba)

    t_i = 1
    mvr_time = 5
    T = 0.01
    t = np.arange(0, 10 + T, T)

    P_i = [1, 1, 1, 0, 0, 0, 1]
    P_f = [5, 3, 4, 1, 1, 1, 0]

    x_ee = np.zeros_like(t)
    y_ee = np.zeros_like(t)
    z_ee = np.zeros_like(t)
    qx = np.zeros_like(t)
    qy = np.zeros_like(t)
    qz = np.zeros_like(t)
    qw = np.zeros_like(t)

    # Simulación de trayectoria "online"
    for k in range(len(t)):
        X_ee = rest_to_rest_trajectory(P_i, P_f, t_i, mvr_time, t[k], 2)
        x_ee[k] = X_ee[0]
        y_ee[k] = X_ee[1]
        z_ee[k] = X_ee[2]
        qx[k] = X_ee[3]
        qy[k] = X_ee[4]
        qz[k] = X_ee[5]
        qw[k] = X_ee[6]

    # ==========================
    # Gráfica
    # ==========================
    plt.plot(t, x_ee, label='x')
    plt.plot(t, y_ee, label='y')
    plt.plot(t, z_ee, label='z')
    plt.plot(t, qx, label='qx')
    plt.plot(t, qy, label='qy')
    plt.plot(t, qz, label='qz')
    plt.plot(t, qw, label='qw')
    plt.title("Pose vs Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Position")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
