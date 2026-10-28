import numpy as np

obstacles = [
    ("gate_left_p", "cyl", np.array([-1.6, 5.0]), 0.2),
    ("gate_right_p", "cyl", np.array([1.6, 5.0]), 0.2),
    ("gate_bar", "box", np.array([0.0, 5.0]), np.array([1.8, 0.2]), 3.85, 4.15),
    ("rooftop", "box", np.array([6.0, 0.0]), np.array([1.6, 1.6]), 0.0, 2.0),
    ("tower_a", "box", np.array([7.0, -6.0]), np.array([2.0, 2.0]), 0.0, 7.5),
    ("tower_b", "box", np.array([-6.0, -6.0]), np.array([1.75, 2.25]), 0.0, 6.0),
    ("pylon_1", "cyl", np.array([4.0, 4.0]), 0.35),
    ("pylon_2", "cyl", np.array([-4.0, 4.0]), 0.35),
    ("pylon_3", "cyl", np.array([-5.0, 0.0]), 0.35),
    ("pylon_4", "cyl", np.array([0.0, -4.0]), 0.35),
]

waypoints = [
    [0.0, 0.0, 2.2],
    [0.0, 3.0, 2.2],
    [0.0, 6.5, 2.2],
    [3.0, 6.5, 2.5],
    [6.8, 4.5, 3.2],
    [6.0, 0.0, 3.5],
    [3.0, -2.5, 3.0],
    [0.0, -2.2, 2.5],
    [-2.8, -2.5, 2.5],
    [-2.0, 0.0, 2.2],
    [0.0, 0.0, 2.0],
]

drone_radius = 0.35

all_clear = True
for i in range(len(waypoints) - 1):
    p1 = np.array(waypoints[i])
    p2 = np.array(waypoints[i+1])
    seg_vec = p2 - p1
    min_dist_to_any = 999.0
    closest_obs = ""
    for alpha in np.linspace(0, 1, 100):
        pt = p1 + alpha * seg_vec
        for obs in obstacles:
            name, o_type = obs[0], obs[1]
            if o_type == "cyl":
                c2d = obs[2]
                r_obs = obs[3]
                d = np.linalg.norm(pt[:2] - c2d) - r_obs - drone_radius
            elif o_type == "box":
                c2d = obs[2]
                half = obs[3]
                z_min, z_max = obs[4], obs[5]
                if pt[2] > z_max:
                    dz = pt[2] - z_max
                elif pt[2] < z_min:
                    dz = z_min - pt[2]
                else:
                    dz = 0.0
                dx = max(0.0, abs(pt[0] - c2d[0]) - half[0])
                dy = max(0.0, abs(pt[1] - c2d[1]) - half[1])
                d2d = np.sqrt(dx**2 + dy**2)
                d = np.sqrt(d2d**2 + dz**2) - drone_radius

            if d < min_dist_to_any:
                min_dist_to_any = d
                closest_obs = name

    print(f"Seg {i} -> {i+1}: Min Clearance = {min_dist_to_any:.2f} m (Nearest: {closest_obs})")
    if min_dist_to_any < 0.5:
        all_clear = False

print("\nAll Clear (> 0.5m buffer):", all_clear)
