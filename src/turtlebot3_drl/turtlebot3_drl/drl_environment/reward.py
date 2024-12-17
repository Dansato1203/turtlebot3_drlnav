from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM, ARENA_WIDTH

goal_dist_initial = 0

reward_function_internal = None

#def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
#    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)
#def get_reward(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, robot_x, robot_y, target_x, target_y):
#    return reward_function_internal(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, robot_x, robot_y, target_x, target_y)
def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance, overtake_happened, closest_agent_distance, time_elapsed):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance, overtake_happened, closest_agent_distance, time_elapsed)

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

def get_reward_C(succeed, action_linear, action_angular, goal_dist, goal_angle, obstacle_dist, overtake_happened=False, closest_agent_distance=999.0, time_elapsed=0.0):
    SUCCESS_REWARD = 2500
    COLLISION_PENALTY = 2000
    BASE_STEP_PENALTY = 1.0
    OBSTACLE_THRESHOLD = 0.3
    OBSTACLE_PENALTY = 20.0
    OVERTAKE_BONUS = 50.0  # 追い越し成功時の報酬
    AGENT_DISTANCE_THRESHOLD = 0.4
    TIME_PENALTY_SCALE = 0.1

    # ゴール進捗: -1～1
    r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1
    # ゴール方向を向くほどよい: -abs(goal_angle)
    r_yaw = -1.0 * abs(goal_angle)
    # 大きな角速度はペナルティ
    r_vangular = -1.0 * (action_angular ** 2)

    # 障害物接近ペナルティ
    if obstacle_dist < OBSTACLE_THRESHOLD:
        r_obstacle = -OBSTACLE_PENALTY
    else:
        r_obstacle = 0.0

    # 前進速度報酬（0.22m/s想定）
    MAX_SPEED = 0.5
    speed_ratio = min(action_linear / MAX_SPEED, 1.0)  
    r_vlinear = 2.0 * speed_ratio - 1.0  # [-1,1]

    # 追い越し時報酬
    r_overtake = OVERTAKE_BONUS if overtake_happened else 0.0

    # 他エージェントとの距離評価
    # 近すぎるとペナルティを付与し、十分離れていれば（あまり大きな報酬にする必要はないが）わずかな報酬も可能
    if closest_agent_distance < AGENT_DISTANCE_THRESHOLD:
        r_agent = -50.0 * (AGENT_DISTANCE_THRESHOLD - closest_agent_distance)  # 距離が小さいほど強いペナルティ
    else:
        r_agent = 0.0  # 安全距離を保っている限りはペナルティなし、必要なら微小報酬付与可

    # 終了時報酬
    if succeed == SUCCESS:
        r_terminal = SUCCESS_REWARD
    elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
        r_terminal = -COLLISION_PENALTY
    elif succeed == TIMEOUT:
        r_terminal = -1000.0
    elif succeed == TUMBLE:
        r_terminal = -500.0
    else:
        r_terminal = 0.0

    r_base = -BASE_STEP_PENALTY
    r_time = -TIME_PENALTY_SCALE * time_elapsed

    reward = (r_yaw + r_distance + r_vangular + r_obstacle + r_vlinear + r_overtake + r_agent + r_terminal + r_base + r_time)
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
