import numpy as np
import math


def get_frame_at_hand(config):
    q1 = config["q1"]
    q2 = config["q2"]
    q3 = config["q3"]
    q4 = config["q4"]
    firstElbowOffset = config["firstElbowOffset"]
    L1 = config["L1"]
    L2 = config["L2"]
    L3 = config["L3"]

    frame_at_hand = np.array([[math.cos(q2 + q3 + q4)*math.cos(q1), -math.sin(q1), math.sin(q2 + q3 + q4)*math.cos(q1),  math.cos(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.cos(q1)],
        [math.cos(q2 + q3 + q4)*math.sin(q1),  math.cos(q1), math.sin(q2 + q3 + q4)*math.sin(q1),  math.sin(q1)*(L2*math.cos(q2 + q3) + L1*math.cos(q2)) + L3*math.cos(q2 + q3 + q4)*math.sin(q1)],
        [       -math.sin(q2 + q3 + q4),        0,         math.cos(q2 + q3 + q4), firstElbowOffset - L2*math.sin(q2 + q3) - L1*math.sin(q2) - L3*math.sin(q2 + q3 + q4)],
        [                        0,        0,                         0,                                                                      1]]
    )

    return {
        "matrix": frame_at_hand,
        "position": frame_at_hand[0:3, 3]
    }


if __name__ == "__main__":
    config = {
        "q1": 0, "q2": 0, "q3": 0, "q4": 0,
        "firstElbowOffset": 5, "L1": 10, "L2": 10, "L3": 10,
        "q1_min": 0, "q1_max": 180,
        "q2_min": 0, "q2_max": 180,
        "q3_min": 0, "q3_max": 180,
        "q4_min": 0, "q4_max": 180
    }

    x = get_frame_at_hand(config)

    print(x)