from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM, ARENA_WIDTH

goal_dist_initial = 0

reward_function_internal = None

def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)
#def get_reward(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, robot_x, robot_y, target_x, target_y):
#    return reward_function_internal(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, robot_x, robot_y, target_x, target_y)

def get_reward_A(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
        # [-3.14, 0]
        r_yaw = -1 * abs(goal_angle)

        # [-4, 0]
        r_vangular = -1 * (action_angular**2)

        # [-1, 1]
        r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1

        # [-20, 0]
        if min_obstacle_dist < 0.22:
            r_obstacle = -20
        else:
            r_obstacle = 0

        # [-2 * (2.2^2), 0]
        r_vlinear = -1 * (((0.22 - action_linear) * 10) ** 2)

        reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1

        if succeed == SUCCESS:
            reward += 2500
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 2000
        return float(reward)

def get_reward_B(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, robot_x, robot_y, target_x, target_y):
    gamma = [-1.0, 1.0, 1.6, 1.0, 1.0]  # weight coefficients for each reward term
    v_min = 0  # minimum velocity
    v_max = 0.3  # maximum velocity
    lane_width = -0.75  # Half of the total pathway width for 'lanes'
    L = 1  # total vehicles on the right lane
    n_front = 1  # vehicles in front of the ego vehicle

    # Collision penalty
    if succeed in [COLLISION_OBSTACLE, COLLISION_WALL]:
        r_collision = gamma[0] * 2000
    else:
        r_collision = 0

    if succeed == TIMEOUT:
        r_time = -3000
    else:
        r_time = 0

    # [-3.14, 0]
    r_yaw = -1 * abs(goal_angle)

    # [-4, 0]
    r_vangular = -1 * (action_angular**2)

    r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1

    # Lane keeping bonus
    if 0 <= robot_y <= 0.75:  # Assuming left side of the pathway as the 'lane'
        if len(target_x) > 1 and abs(target_x[0] - robot_x) < 0.5:
            print("1")
            r_lane = gamma[1] * 20
        else:
            r_lane = 0
    else:
        r_lane = 0

    # Velocity bonus
    r_velocity = gamma[2] * ((action_linear - v_min) / (v_max - v_min))

    # Overtaking bonus
    if -0.75 <= robot_y < 0:  # Right side of the pathway for overtaking
        r_overtaking = gamma[3] * (1 if any(tx < robot_x for tx in target_x) else -1) * 20
    else:
        r_overtaking = 0

    # Terminal bonus
    if succeed == SUCCESS:
        r_terminal = 2500
    else:
        r_terminal = 0

    # Combine all rewards
    reward = r_collision + r_lane + r_overtaking + r_terminal + r_yaw + r_vangular + r_time + r_distance + r_velocity

    return float(reward)

# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")
