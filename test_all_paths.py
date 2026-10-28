import math
import numpy as np

mass = 1.50
g = 9.81
ixx, iyy, izz = 0.015, 0.015, 0.025

kp_xy, ki_xy, kd_xy = 5.5, 1.2, 3.8
kp_z, ki_z, kd_z = 8.5, 2.5, 5.2
kp_att, kd_att = 4.5, 0.65

def test_smooth_circle():
    dt = 0.01
    pos = np.array([0.0, 0.0, 2.0])
    vel = np.array([0.0, 0.0, 0.0])
    att = np.array([0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 0.0])
    pos_int = np.array([0.0, 0.0, 0.0])

    R_target = 2.5
    speed = 1.5
    theta = 0.0
    r_curr = 0.0

    errors = []
    for i in range(1200): # 12 seconds
        t = i * dt
        # Smoothly expand radius from 0 to 2.5m over 2.5 seconds
        if r_curr < R_target:
            r_curr = min(R_target, r_curr + (R_target / 2.5) * dt)
            r_dot = (R_target / 2.5)
        else:
            r_dot = 0.0

        omg = speed / R_target if r_curr > 0.1 else 0.0
        theta += omg * dt

        # Analytical position, velocity, acceleration for expanding spiral into circle:
        target_pos = np.array([r_curr * math.cos(theta), r_curr * math.sin(theta), 2.0])
        target_vel = np.array([
            r_dot * math.cos(theta) - r_curr * omg * math.sin(theta),
            r_dot * math.sin(theta) + r_curr * omg * math.cos(theta),
            0.0
        ])
        target_acc = np.array([
            -r_curr * (omg**2) * math.cos(theta) - 2 * r_dot * omg * math.sin(theta),
            -r_curr * (omg**2) * math.sin(theta) + 2 * r_dot * omg * math.cos(theta),
            0.0
        ])

        err_p = target_pos - pos
        err_v = target_vel - vel
        pos_int += err_p * dt
        pos_int = np.clip(pos_int, -1.0, 1.0)

        ax_des = target_acc[0] + kp_xy * err_p[0] + ki_xy * pos_int[0] + kd_xy * err_v[0]
        ay_des = target_acc[1] + kp_xy * err_p[1] + ki_xy * pos_int[1] + kd_xy * err_v[1]
        az_des = target_acc[2] + kp_z  * err_p[2] + ki_z  * pos_int[2] + kd_z  * err_v[2] + g

        max_tilt = math.radians(35.0)
        phi_des = np.clip((ax_des * math.sin(att[2]) - ay_des * math.cos(att[2])) / g, -max_tilt, max_tilt)
        theta_des = np.clip((ax_des * math.cos(att[2]) + ay_des * math.sin(att[2])) / g, -max_tilt, max_tilt)
        psi_des = 0.0
        total_thrust = np.clip(mass * az_des, 0.0, 2.5 * mass * g)

        sub_steps = 5
        sub_dt = dt / sub_steps
        for _ in range(sub_steps):
            tau_x = kp_att * (phi_des - att[0]) - kd_att * omega[0]
            tau_y = kp_att * (theta_des - att[1]) - kd_att * omega[1]
            tau_z = 2.0 * (psi_des - att[2]) - 0.3 * omega[2]

            cr, sr = math.cos(att[0]), math.sin(att[0])
            cp, sp = math.cos(att[1]), math.sin(att[1])
            cy, sy = math.cos(att[2]), math.sin(att[2])
            R_b_w = np.array([
                [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                [-sp,   sr*cp,            cr*cp]
            ])
            thrust_w = R_b_w @ np.array([0.0, 0.0, total_thrust])
            a_tot = thrust_w / mass + np.array([0.0, 0.0, -g])
            alpha_ang = np.array([tau_x / ixx, tau_y / iyy, tau_z / izz])

            vel += a_tot * sub_dt
            pos += vel * sub_dt
            omega += alpha_ang * sub_dt
            att += omega * sub_dt

        err = np.linalg.norm(target_pos - pos)
        errors.append(err)

    print("Smooth Circle Flight (Spiral Ingress + Steady Orbit):")
    print("  Overall Mean Error:", round(float(np.mean(errors)) * 100, 2), "cm")
    print("  Max Transient Error:", round(float(np.max(errors)) * 100, 2), "cm")
    print("  Steady-State Orbit Mean Error (t > 4s):", round(float(np.mean(errors[400:])) * 100, 2), "cm")

test_smooth_circle()
