import pybullet as p
import numpy as np

TOKEN_REWARD = 10
DEAD_END_PENALTY = -5
TIME_STEP_PENALTY = -0.1

#define as list of tuples
tokens = [] 
dead_ends = []
goal_position = () #just one tuple here

# create walls, start, and end points of the maze
def create_maze():
    pass

# create the robot model
def create_robot():
    pass

def calculate_reward(state, next_state, reached_goal):
    reward = TIME_STEP_PENALTY
    if is_token(next_state[pos]):
        reward += TOKEN_REWARD
    elif is_dead_end(next_state[pos]):
        reward += DEAD_END_PENALTY
    elif reached_goal(next_state[pos]):
        reward += 10
    return reward

def is_token(position):
    return position in tokens

#TODO: modify so it's the postion and orientation (it's not a dead end if you are turning trying to get out of it)
def is_dead_end(position):
    return position in dead_ends

def def reached_goal(position)
    return position == goal_postion

def observation_to_state(observation):
    position, orientation = observation
    state = {pos: postition, dir: orientation}
    return state

def take_action(action):
    # TODO: tell the robot to do something
    observation = p.getBasePositionAndOrientation(robot)
    next_state = observation_to_state(observation)
    reward = calculate_reward(state, next_state, goal_reached)
    return next_state, reward, goal_reached

def choose_action(state, Q, epsilon):
    random_num = random.uniform(0, 1)
    if state not in Q or andom.uniform(0, 1) < epsilon:
        # Explore!!
        action = random.randint(0, num_actions - 1)
    else:
        action = np.argmax(Q[state])
    return action

def update_Q(Q, state, action, reward, next_state, alpha, gamma):
    if Q[state][action] is None:
        Q[state][action] = 1
    current_Q = Q[state][action]
    max_next_Q = np.max(Q[next_state])
    new_Q = current_Q + alpha * (reward + gamma * max_next_Q - current_Q)
    Q[state][action] = new_Q

# Training params
epsilon = 0.1
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor

num_states = 100
num_actions = 4  # (left, right, forward, backwards)
Q = np.zeros((num_states, num_actions))
num_episodes = 1000

p.connect(p.GUI)  # Connect to the physics server in GUI mode

for _ in range(num_episodes):
    p.resetSimulation()
    maze = create_maze()
    robot = create_robot()
    
    # Initial state
    observation = p.getBasePositionAndOrientation(robot)
    state = observation_to_state(observation)
    
    while not reached_goal:
        action = choose_action(state, Q, epsilon)
        next_state, reward, reached_goal = take_action(action)
        update_Q(Q, state, action, reward, next_state, alpha, gamma)
        state = next_state

p.disconnect()
