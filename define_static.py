def define(shared_state, shared_lock, pi):
    global_state = {}

    global_state["lengths"] = { "firstElbowOffset": 0, "L1": 20, "L2": 20, "L3": 10 }

    global_state["q pinout"] = {
        "q1": 19,
        "q2": 26,
        "q3": 16,
        "q4": 18,
        "q5": 25
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

    global_state["q degrees bounds"] = {
        "q1": (-72, 108),
        "q2": (-19, 181),
        "q3":  (-165.6, 14.4),
        "q4": (-177.75, 2.25),
        "q5": (0, 1)
    }

    global_state["set position lambda"] = {
        "q1": lambda x: 1700 - x * 2000/180,
        "q2": lambda x: 1590 - (90 - x) * 2000/200,
        "q3": lambda x: 660 - x * 2000/180,
        "q4": lambda x: 1475 + (x + 90) * 2000/180,
        "q5": lambda x: (2250-1290) * (1-x) + 1290
    }

    return global_state