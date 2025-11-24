import numpy as np
import math
import random


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

    def sign(intval):
        if intval >= 0:
            return 1
        return -1

    for _ in range(iterations):
        candidate = []
        for (xi, (a, b)) in zip(x, bounds):
            step = step_scale * sign((random.random() - 0.5))
            new_val = xi + step

            new_val = max(a, min(b, new_val))
            candidate.append(new_val)

        c = cost_func(candidate)

        if c < best_cost:
            best_cost = c
            x = candidate

    return x, best_cost


def numberical_solution(init_q_config, static_config, q_bounds, desired_hand_position,
                        gradient_iterations=10, gradient_iterations_per_optimization=15, gradient_a = 0.01,
                        custom_optimization_iterations_per_gradient = 100, custom_optimization_step = 0.01):
    if gradient_iterations == 0:
        raise Exception("Must be grater than 0")
        return

    def cost_func(q_config):
        return np.linalg.norm(desired_hand_position - get_frame_at_hand(q_config, static_config)['position'])

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
    found_c = cost_func(found_q)
    best_solutions.append((found_q, found_c))

    for _ in range(gradient_iterations-1):
        found_q = clamp_to_bounds(gradient_optimization(
            np.array([random.uniform(a, b) for (a, b) in q_bounds]),
            static_config,
            desired_hand_position,
            iterations=gradient_iterations_per_optimization,
            alpha=gradient_a
        ))
        found_c = cost_func(found_q)
        best_solutions.append((found_q, found_c))


    for i in range(len(best_solutions)):
        (q, c) = custom_optimize(
            starting_parameters=best_solutions[i][0],
            cost_func=cost_func,
            bounds=q_bounds,
            iterations=custom_optimization_iterations_per_gradient,
            step_scale=custom_optimization_step
        )
        print(best_solutions[i][1], "--->", c)
        best_solutions[i] = (q, c)

    best_idx = None
    best_c = None
    for i in range(len(best_solutions)):
        if best_idx == None:
            best_idx = i
            best_c = best_solutions[i][1]
        if best_c > best_solutions[i][1]:
            best_idx = i
            best_c = best_solutions[i][1]

    return {
        "best_q": best_solutions[best_idx][0],
        "cost_q": best_solutions[best_idx][1]
    }



if __name__ == "__main__":
    static_config = {
        "firstElbowOffset": 5, "L1": 10, "L2": 10, "L3": 10
    }

    init_q_config = np.array([0, 0, 0, 0]) # q1, q2, q3, q4    
    q_bounds = [
        (0, 180), # q1
        (0, 180), # q2
        (0, 180), # q3
        (0, 180)  # q4
    ]

    desired_hand_position = np.array([5, 2, 1])
    
    x = numberical_solution(
        init_q_config,
        static_config,
        q_bounds,
        desired_hand_position,
        gradient_iterations=15, gradient_iterations_per_optimization=15, gradient_a = 0.01,
        custom_optimization_iterations_per_gradient = 100, custom_optimization_step = 0.01
    )
    print(x)
    print(get_frame_at_hand(x['best_q'], static_config)['position'])
    