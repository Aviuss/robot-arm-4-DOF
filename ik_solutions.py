import numpy as np
import math
import random

def deg2rad(deg):
    return deg * math.pi / 180

def rad2deg(rad):
    return rad * 180 / math.pi


def robot_graph(q, static_config):
    import matplotlib.pyplot as plt

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    L1 = static_config["L1"]
    L2 = static_config["L2"]
    L3 = static_config["L3"]
    firstElbowOffset = static_config["firstElbowOffset"]

    def rotz(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0,              0,             1, 0],
            [0,              0,             0, 1]
        ])

    def roty(theta):
        return np.array([
            [ np.cos(theta), 0, np.sin(theta), 0],
            [ 0,             1, 0,             0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [ 0,             0, 0,             1]
        ])

    def trans(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])


    # Forward kinematics
    T0 = np.eye(4)
    T1 = T0 @ rotz(q1)
    #frame_base_rotated_arm = simplify(frame_base * HomTrans(ROTATIONAL_MATRIX_Z(q1), [0;0;0]));
    T2 = T1 @ trans(0, 0, firstElbowOffset) @ roty(q2)
    #frame_at_elbow_1 = simplify(frame_base_rotated_arm * HomTrans(ROTATIONAL_MATRIX_Z(0), [0;0;firstElbowOffset]));
    #frame_rotated_elbow_1 = simplify(frame_at_elbow_1 * HomTrans(ROTATIONAL_MATRIX_Y(q2), [0;0;0]));
    T3 = T2 @ trans(L1, 0, 0) @ roty(q3)
    #frame_at_elbow_2 = simplify(frame_rotated_elbow_1 * HomTrans(ROTATIONAL_MATRIX_Z(0), [L1;0;0]));
    #frame_rotated_elbow_2 = simplify(frame_at_elbow_2 * HomTrans(ROTATIONAL_MATRIX_Y(q3), [0;0;0]));
    T4 = T3 @ trans(L2, 0, 0) @ roty(q4)
    #frame_at_elbow_3 = simplify(frame_rotated_elbow_2 * HomTrans(ROTATIONAL_MATRIX_Z(0), [L2;0;0]));
    #frame_rotated_elbow_3 = simplify(frame_at_elbow_3 * HomTrans(ROTATIONAL_MATRIX_Y(q4), [0;0;0]));
    T_hand = T4 @ trans(L3, 0, 0)
    #frame_at_hand = simplify(frame_rotated_elbow_3 * HomTrans(ROTATIONAL_MATRIX_Z(0), [L3;0;0]))

    # Collect joint positions
    points = np.array([
        T0[:3, 3],
        T1[:3, 3],
        T2[:3, 3],
        T3[:3, 3],
        T4[:3, 3],
        T_hand[:3, 3]
    ])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Hand Kinematic Chain')

    # 🔥 force equal scaling
    x_limits = [points[:,0].min(), points[:,0].max()]
    y_limits = [points[:,1].min(), points[:,1].max()]
    z_limits = [points[:,2].min(), points[:,2].max()]

    x_mid = np.mean(x_limits)
    y_mid = np.mean(y_limits)
    z_mid = np.mean(z_limits)

    max_range = max(
        x_limits[1] - x_limits[0],
        y_limits[1] - y_limits[0],
        z_limits[1] - z_limits[0]
    ) / 2

    ax.set_xlim(x_mid - max_range, x_mid + max_range)
    ax.set_ylim(y_mid - max_range, y_mid + max_range)
    ax.set_zlim(z_mid - max_range, z_mid + max_range)

    plt.show()



def get_frame_at_hand(q, static_config):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    firstElbowOffset = static_config["firstElbowOffset"]
    L1 = static_config["L1"]
    L2 = static_config["L2"]
    L3 = static_config["L3"]

    frame_at_hand = np.array([[math.cos(q2 + q3 + q4)*math.cos(q1), -math.sin(q1), math.sin(q2 + q3 + q4)*math.cos(q1),  math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.cos(q1)],
        [math.cos(q2 + q3 + q4)*math.sin(q1),  math.cos(q1), math.sin(q2 + q3 + q4)*math.sin(q1),  math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.sin(q1)],
        [       -math.sin(q2 + q3 + q4),        0,         math.cos(q2 + q3 + q4), firstElbowOffset - L2*math.sin(q2 + q3) - L1*math.sin(q2) - L3*math.sin(q2 + q3 + q4)],
        [                        0,        0,                         0,                                                                      1]]
    )

    return {
        "matrix": frame_at_hand,
        "position": frame_at_hand[0:3, 3]
    }

def analytical_jacobian_at_hand(q, static_config):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    firstElbowOffset = static_config["firstElbowOffset"]
    L1 = static_config["L1"]
    L2 = static_config["L2"]
    L3 = static_config["L3"]


    jac = np.array([[- math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) - L3*math.cos(q2 + q3 + q4)*math.sin(q1), - math.cos(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - L3*math.sin(q2 + q3 + q4)*math.cos(q1), -math.cos(q1)*(L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)), -L3*math.sin(q2 + q3 + q4)*math.cos(q1)],
        [  math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.cos(q1), - math.sin(q1)*(L2*math.sin(q2 + q3) + L1*math.sin(q2)) - L3*math.sin(q2 + q3 + q4)*math.sin(q1), -math.sin(q1)*(L2*math.sin(q2 + q3) + L3*math.sin(q2 + q3 + q4)), -L3*math.sin(q2 + q3 + q4)*math.sin(q1)],
        [                                                                      0,                   - L2*math.cos(q2 + q3) - L1*math.cos(q2) - L3*math.cos(q2 + q3 + q4),          - L2*math.cos(q2 + q3) - L3*math.cos(q2 + q3 + q4),         -L3*math.cos(q2 + q3 + q4)]]) 

    return jac

def gradient_optimization(init_q, static_config, desired_hand_position, iterations = 10, alpha = 0.05):
    for _ in range(iterations):
        jac_t = np.transpose(analytical_jacobian_at_hand(init_q, static_config))
        error0 = (desired_hand_position - get_frame_at_hand(init_q, static_config)['position'])
        init_q = init_q + alpha * np.dot(jac_t, error0)
    
    return init_q

def custom_optimize(starting_parameters, cost_func, bounds, iterations=200, step_scale=0.1):
    x = starting_parameters.copy()
    best_cost = cost_func(x)

    for _ in range(iterations):
        candidate = []
        for (xi, (a, b)) in zip(x, bounds):
            step = step_scale * (random.random() * 2  - 1)
            new_val = xi + step

            new_val = max(a, min(b, new_val))
            candidate.append(new_val)

        candidate = np.array(candidate)
        c = cost_func(candidate, False)

        if c < best_cost:
            best_cost = c
            x = candidate

    return x, best_cost


def numberical_solution(init_q_config, static_config, q_bounds, desired_hand_position, q_preffered,
                        gradient_iterations=10, gradient_iterations_per_optimization=15, gradient_a = 0.01,
                        custom_optimization_iterations_per_gradient = 100, custom_optimization_step = 0.01, class_round_val = 5, take_max_for_scnd_optimisation = 20):
    if gradient_iterations == 0:
        raise Exception("Must be grater than 0")
        return

    
    def class_round(integer):
        return round(integer / class_round_val) * class_round_val

    def cost_angle(q_config):
        angle_cost = np.sum((q_config - q_preffered)**2)
        return angle_cost

    def cost_func(q_config, class_round_ok = True):
        pos_cost = np.linalg.norm(desired_hand_position - get_frame_at_hand(q_config, static_config)['position'])
        
        if class_round_ok:
            return class_round(pos_cost)

        return pos_cost

    def clamp_to_bounds(q):
        res = []
        for (xi, (a, b)) in zip(q, q_bounds):
            xi = max(a, min(b, xi))
            res.append(xi)
        return np.array(res)

    best_solutions = []

    found_q = clamp_to_bounds(gradient_optimization(
        init_q_config.copy(),
        static_config,
        desired_hand_position,
        iterations=gradient_iterations_per_optimization,
        alpha=gradient_a
    ))
    best_solutions.append([found_q, cost_func(found_q), cost_angle(found_q)])

    if (best_solutions[0][1] == 0):
        (q, c) = custom_optimize(
            starting_parameters=best_solutions[0][0],
            cost_func=cost_func,
            bounds=q_bounds,
            iterations=custom_optimization_iterations_per_gradient,
            step_scale=custom_optimization_step
        )

        return {
            "best_q": q,
            "cost_q": c
        } 


    for _ in range(gradient_iterations-1):
        found_q = clamp_to_bounds(gradient_optimization(
            np.array([random.uniform(a, b) for (a, b) in q_bounds]),
            static_config,
            desired_hand_position,
            iterations=gradient_iterations_per_optimization,
            alpha=gradient_a
        ))
        
        best_solutions.append([found_q, cost_func(found_q), cost_angle(found_q)])

    best_solutions.sort(key=lambda x:[x[1], x[2]])

    best_solutions = best_solutions[0: min(take_max_for_scnd_optimisation, len(best_solutions)) ]

    for i in range(len(best_solutions)):
        (q, c) = custom_optimize(
            starting_parameters=best_solutions[i][0],
            cost_func=cost_func,
            bounds=q_bounds,
            iterations=custom_optimization_iterations_per_gradient,
            step_scale=custom_optimization_step
        )
        best_solutions[i][1] = c
        

    best_idx = None
    best_c = None
    for i in range(len(best_solutions)):
        if best_solutions[i] == None:
            continue
        if best_idx == None:
            best_idx = i
            best_c = best_solutions[i][1]
        if best_c > best_solutions[i][1]:
            best_idx = i
            best_c = best_solutions[i][1]

    for i in range(len(best_solutions)):
        if best_solutions[i] == None:
            continue
        print(list(map(lambda x:round(rad2deg(x)*10)/10, best_solutions[i][0])), round(best_solutions[i][1]*1000)/1000, end="")
        if i == best_idx:
            print(" chosen")
        else:
            print()


    return {
        "best_q": best_solutions[best_idx][0],
        "cost_q": best_solutions[best_idx][1]
    }


if __name__ == "__main__":
    static_config = {
        "firstElbowOffset": 0, "L1": 20, "L2": 20, "L3": 16
    }

    init_q_config = np.array([0, 0, 0, 0]) # q1, q2, q3, q4    
    q_bounds = [
        (deg2rad(-72), deg2rad(108)), # q1
        (deg2rad(-19), deg2rad(181)), # q2
        (deg2rad(-165.6), deg2rad(14.4)), # q3
        (deg2rad(-177.75), deg2rad(2.25)), # q4
    ]
    desired_hand_position = np.array([45, 0, 0])
    
    x = numberical_solution(
        init_q_config,
        static_config,
        q_bounds,
        desired_hand_position,
        q_preffered = np.array([deg2rad(0), deg2rad(90), deg2rad(0), deg2rad(0)]),
        gradient_iterations=50, gradient_iterations_per_optimization=200, gradient_a = 0.0001,
        class_round_val = 3,
        
        take_max_for_scnd_optimisation = 10,
        custom_optimization_iterations_per_gradient = 1000, custom_optimization_step = 0.05
    )
    print(get_frame_at_hand(x['best_q'], static_config)['position'])
    print(x, "\n", list(map(lambda x: round(rad2deg(x)*100)/100, x["best_q"])))
    robot_graph(x["best_q"], static_config)