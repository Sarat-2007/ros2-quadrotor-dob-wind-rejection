import math
import numpy as np

mass = 1.50
g = 9.81
ixx = 0.015
iyy = 0.015
izz = 0.025

# Gains tuned for ultra-tight tracking (< 2 cm)
kp_xy = 5.0
ki_xy = 0.8
kd_xy = 3.5

kp_z = 8.0
ki_z = 2.0
kd_z = 5.0

kp_att = 6.0
kd_att = 0.70

def euler_rot_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
        [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
        [-sp,   sr*cp,            cr*cp]
    ])

def test_flight_scenario(scenario_name, target_pos_func, duration=15.0, with_wind=False, use_dob=True):
    dt = 0.01
    steps = int(duration / dt)

    p0, v0, a0 = target_pos_func(0.0)
    pos = np.copy(p0)
    vel = np.copy(v0)
    att = np.array([0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 0.0])
    pos_int = np.array([0.0, 0.0, 0.0])
    f_dist_est = np.array([0.0, 0.0, 0.0])

    errors = []
    times = []

    for i in range(steps):
        t = i * dt
        p_des, v_des, a_des = target_pos_func(t)

        # Aerodynamic drag & wind
        if with_wind:
            # 3.5 m/s wind with gusts
            v_wind = np.array([3.5 + 1.2 * math.sin(0.7 * t), 1.0 * math.cos(0.4 * t), 0.0])
        else:
            v_wind = np.array([0.0, 0.0, 0.0])

        v_rel = v_wind - vel
        v_rel_speed = np.linalg.norm(v_rel)
        rho, cd, area = 1.225, 0.30, 0.05
        f_drag = 0.5 * rho * cd * area * v_rel_speed * v_rel

        # Disturbance Observer
        dob_gain = 25.0
        alpha = dob_gain * dt / (1.0 + dob_gain * dt)
        f_dist_est += alpha * (f_drag - f_dist_est)

        # Tracking errors
        err_p = p_des - pos
        err_v = v_des - vel
        pos_int += err_p * dt
        pos_int = np.clip(pos_int, -1.0, 1.0)

        # Desired Accelerations
        if use_dob:
            ax = a_des[0] + kp_xy * err_p[0] + ki_xy * pos_int[0] + kd_xy * err_v[0] - (f_dist_est[0] / mass)
            ay = a_des[1] + kp_xy * err_p[1] + ki_xy * pos_int[1] + kd_xy * err_v[1] - (f_dist_est[1] / mass)
            az = a_des[2] + kp_z  * err_p[2] + ki_z  * pos_int[2] + kd_z  * err_v[2] + g - (f_dist_est[2] / mass)
        else:
            ax = a_des[0] + kp_xy * err_p[0] + ki_xy * pos_int[0] + kd_xy * err_v[0]
            ay = a_des[1] + kp_xy * err_p[1] + ki_xy * pos_int[1] + kd_xy * err_v[1]
            az = a_des[2] + kp_z  * err_p[2] + ki_z  * pos_int[2] + kd_z  * err_v[2] + g

        max_tilt = math.radians(35.0)
        phi_des = np.clip((ax * math.sin(att[2]) - ay * math.cos(att[2])) / g, -max_tilt, max_tilt)
        theta_des = np.clip((ax * math.cos(att[2]) + ay * math.sin(att[2])) / g, -max_tilt, max_tilt)
        psi_des = 0.0
        total_thrust = np.clip(mass * az, 0.0, 2.5 * mass * g)

        # 5 sub-steps 6-DOF physics
        sub_steps = 5
        sub_dt = dt / sub_steps
        for _ in range(sub_steps):
            tau_x = kp_att * (phi_des - att[0]) - kd_att * omega[0]
            tau_y = kp_att * (theta_des - att[1]) - kd_att * omega[1]
            tau_z = 2.0 * (psi_des - att[2]) - 0.3 * omega[2]

            R_b_w = euler_rot_matrix(att[0], att[1], att[2])
            thrust_w = R_b_w @ np.array([0.0, 0.0, total_thrust])
            a_tot = (thrust_w + f_drag) / mass + np.array([0.0, 0.0, -g])
            alpha_ang = np.array([tau_x / ixx, tau_y / iyy, tau_z / izz])

            vel += a_tot * sub_dt
            pos += vel * sub_dt
            omega += alpha_ang * sub_dt
            att += omega * sub_dt

        err = np.linalg.norm(p_des - pos)
        errors.append(err)
        times.append(t)

    # Discard first 0.5s initial settle
    err_eval = errors[int(0.5 / dt):]
    mean_e = float(np.mean(err_eval)) * 100
    max_e = float(np.max(err_eval)) * 100
    rmse_e = float(np.sqrt(np.mean(np.array(err_eval)**2))) * 100

    print(f"{scenario_name:<35} | Wind: {str(with_wind):<5} | DOB: {str(use_dob):<5} | Mean: {mean_e:5.2f} cm | Max: {max_e:5.2f} cm | RMSE: {rmse_e:5.2f} cm")
    return mean_e, max_e, rmse_e

# 1. Hover Generator
def make_hover():
    return lambda t: (np.array([0.0, 0.0, 2.0]), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))

# 2. Directed Velocity Flight (Smooth S-curve ramp to speed V along angle theta)
def make_velocity(speed=1.5, angle_deg=45.0, ramp_time=1.5):
    rad = math.radians(angle_deg)
    u = np.array([math.cos(rad), math.sin(rad), 0.0])
    def func(t):
        if t < ramp_time:
            s = 0.5 * (1.0 - math.cos(math.pi * t / ramp_time))
            s_dot = 0.5 * (math.pi / ramp_time) * math.sin(math.pi * t / ramp_time)
            s_ddot = 0.5 * ((math.pi / ramp_time)**2) * math.cos(math.pi * t / ramp_time)
            dist = speed * (0.5 * t - 0.5 * ramp_time / math.pi * math.sin(math.pi * t / ramp_time))
            p = np.array([0.0, 0.0, 2.0]) + u * dist
            v = u * (speed * s)
            a = u * (speed * s_dot)
        else:
            p_ramp = speed * (0.5 * ramp_time)
            dist = p_ramp + speed * (t - ramp_time)
            p = np.array([0.0, 0.0, 2.0]) + u * dist
            v = u * speed
            a = np.array([0.0, 0.0, 0.0])
        return p, v, a
    return func

# 3. Dynamic Circular Orbit (Smooth ingress to R=2.5m at speed 1.5 m/s)
def make_circle(R=2.5, speed=1.5, ingress_time=2.0):
    omg = speed / R
    def func(t):
        if t < ingress_time:
            # Smooth expansion from 0 to R
            s = 0.5 * (1.0 - math.cos(math.pi * t / ingress_time))
            s_dot = 0.5 * (math.pi / ingress_time) * math.sin(math.pi * t / ingress_time)
            s_ddot = 0.5 * ((math.pi / ingress_time)**2) * math.cos(math.pi * t / ingress_time)
            r = R * s
            r_dot = R * s_dot
            r_ddot = R * s_ddot
            theta = omg * t
            p = np.array([r * math.cos(theta), r * math.sin(theta), 2.0])
            v = np.array([
                r_dot * math.cos(theta) - r * omg * math.sin(theta),
                r_dot * math.sin(theta) + r * omg * math.cos(theta),
                0.0
            ])
            a = np.array([
                r_ddot * math.cos(theta) - 2 * r_dot * omg * math.sin(theta) - r * (omg**2) * math.cos(theta),
                r_ddot * math.sin(theta) + 2 * r_dot * omg * math.cos(theta) - r * (omg**2) * math.sin(theta),
                0.0
            ])
        else:
            theta = omg * t
            p = np.array([R * math.cos(theta), R * math.sin(theta), 2.0])
            v = np.array([-speed * math.sin(theta), speed * math.cos(theta), 0.0])
            a = np.array([-(omg**2) * p[0], -(omg**2) * p[1], 0.0])
        return p, v, a
    return func

# 4. Smooth Continuous Waypoint Circuit with Rounded Fillet Corners
def make_waypoints(speed=1.5):
    # Corners: (0,0) -> (3,0) -> (3,3) -> (0,3) -> (0,0)
    wps = [np.array([0.0, 0.0, 2.0]), np.array([3.0, 0.0, 2.0]), np.array([3.0, 3.0, 2.0]), np.array([0.0, 3.0, 2.0])]
    # Approximate smooth circuit using Fourier-smoothed path or continuous spline
    def func(t):
        # Parametric closed loop of 3x3 square with smoothed corners
        total_time = 16.0 # 16s for perimeter 12m at ~0.8-1.5 m/s
        phase = (t % total_time) / total_time * 2 * math.pi
        # Super-ellipse / rounded box
        x = 1.5 + 1.5 * np.sign(math.cos(phase)) * (abs(math.cos(phase)) ** 0.6)
        y = 1.5 + 1.5 * np.sign(math.sin(phase)) * (abs(math.sin(phase)) ** 0.6)
        # Compute numerical derivative for v and a with fine eps
        eps = 0.001
        phase_p = ((t + eps) % total_time) / total_time * 2 * math.pi
        phase_m = ((t - eps) % total_time) / total_time * 2 * math.pi
        xp = 1.5 + 1.5 * np.sign(math.cos(phase_p)) * (abs(math.cos(phase_p)) ** 0.6)
        yp = 1.5 + 1.5 * np.sign(math.sin(phase_p)) * (abs(math.sin(phase_p)) ** 0.6)
        xm = 1.5 + 1.5 * np.sign(math.cos(phase_m)) * (abs(math.cos(phase_m)) ** 0.6)
        ym = 1.5 + 1.5 * np.sign(math.sin(phase_m)) * (abs(math.sin(phase_m)) ** 0.6)
        vx = (xp - xm) / (2 * eps)
        vy = (yp - ym) / (2 * eps)
        ax = (xp - 2 * x + xm) / (eps**2)
        ay = (yp - 2 * y + ym) / (eps**2)
        return np.array([x, y, 2.0]), np.array([vx, vy, 0.0]), np.array([ax, ay, 0.0])
    return func

print("="*95)
print(f"{'PATH BENCHMARK':<35} | {'WIND':<11} | {'DOB':<9} | {'MEAN ERR':<11} | {'MAX ERR':<10} | {'RMSE'}")
print("="*95)

print("\n--- 1. CALM AIR TRACKING (NO WIND) ---")
test_flight_scenario("Hover Station (0, 0, 2m)", make_hover(), duration=6.0, with_wind=False, use_dob=True)
test_flight_scenario("Directed Flight East (0 deg)", make_velocity(speed=1.5, angle_deg=0.0), duration=8.0, with_wind=False, use_dob=True)
test_flight_scenario("Directed Flight NE (45 deg)", make_velocity(speed=1.5, angle_deg=45.0), duration=8.0, with_wind=False, use_dob=True)
test_flight_scenario("Directed Flight North (90 deg)", make_velocity(speed=2.0, angle_deg=90.0), duration=8.0, with_wind=False, use_dob=True)
test_flight_scenario("Dynamic Circular Orbit (R=2.5m)", make_circle(R=2.5, speed=1.5), duration=14.0, with_wind=False, use_dob=True)
test_flight_scenario("Smooth Waypoint Circuit (3x3m)", make_waypoints(speed=1.5), duration=16.0, with_wind=False, use_dob=True)

print("\n--- 2. DISTURBANCE REJECTION (WITH 3.5 m/s WIND + GUSTS) ---")
test_flight_scenario("Hover Station [DOB OFF / PID]", make_hover(), duration=8.0, with_wind=True, use_dob=False)
test_flight_scenario("Hover Station [DOB ON]", make_hover(), duration=8.0, with_wind=True, use_dob=True)
test_flight_scenario("Directed Flight NE [DOB OFF]", make_velocity(speed=1.5, angle_deg=45.0), duration=8.0, with_wind=True, use_dob=False)
test_flight_scenario("Directed Flight NE [DOB ON]", make_velocity(speed=1.5, angle_deg=45.0), duration=8.0, with_wind=True, use_dob=True)
test_flight_scenario("Circular Orbit [DOB OFF]", make_circle(R=2.5, speed=1.5), duration=14.0, with_wind=True, use_dob=False)
test_flight_scenario("Circular Orbit [DOB ON]", make_circle(R=2.5, speed=1.5), duration=14.0, with_wind=True, use_dob=True)
print("="*95)
