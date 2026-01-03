def define(shared_state, shared_lock, pi):
    global_state = {}

    global_state["q pinout"] = {
        "q1": 19,
        "q2": 13,
        "q3": 12,
        "q4": 18,
        "q5": 23
    }
    global_state["q pulse restriction"] = {
        "q1": (500, 2500),
        "q2": (500, 2500),
        "q3": (500, 2500),
        "q4": (500, 2500),
        "q5": (1290, 2250)
    }
    global_state["variables"] = {
        "shared": {
            "state": shared_state,
            "lock": shared_lock
        },
        "pi": pi
    }

    return global_state