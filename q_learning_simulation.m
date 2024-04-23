TOKEN_REWARD = 10;
DEAD_END_PENALTY = -5;
TIME_STEP_PENALTY = -0.1;

function reward = calculate_reward(state, next_state, reached_goal)
    reward = TIME_STEP_PENALTY;
    if is_token(next_state(1)) % TODO: needs to be modified with the state
        reward = reward + TOKEN_REWARD;
    elseif is_dead_end(next_state(1)) % TODO: ^^ here too
        reward = reward + DEAD_END_PENALTY;
    elseif reached_goal(next_state(1)) % TODO: ^^ here too
        reward = reward + 10;
    end
end

function result = is_token(position)
    result = ismember(position, tokens);
end

% TODO: Modify so it's the position and orientation (it's not a dead end if you are turning trying to get out of it)
function result = is_dead_end(position)
    result = ismember(position, dead_ends);
end

function result = reached_goal(position)
    result = isequal(position, goal_position);
end

function state = observation_to_state(observation)
    position = observation(1); % Assuming position is the first element of the observation array
    orientation = observation(2); % Assuming orientation is the second element of the observation array
    state = [position, orientation];
end

function [next_state, reward, reached_goal] = take_action(action)
    % TODO: Tell the robot to do something
    observation = getBasePositionAndOrientation(robot);
    next_state = observation_to_state(observation);
    reward = calculate_reward(state, next_state, goal_reached);
end

function action = choose_action(state, Q, epsilon)
    random_num = rand();
    if rand_num < epsilon || ~isfield(Q, state)
        % Explore!!
        action = randi([1, num_actions]);
    else
        [~, action] = max(Q(state, :));
    end
end

function update_Q(Q, state, action, reward, next_state, alpha, gamma)
    if isempty(Q(state, action))
        Q(state, action) = 1;
    end
    current_Q = Q(state, action);
    max_next_Q = max(Q(next_state, :));
    new_Q = current_Q + alpha * (reward + gamma * max_next_Q - current_Q);
    Q(state, action) = new_Q;
end


epsilon = 0.1;
alpha = 0.1;
gamma = 0.9;

num_states = 100;
num_actions = 4;  % (left, right, forward, backwards) TODO: right? is backwards necessary

if exist('Q.mat', 'file') == 2
    load('Q.mat', 'Q');
else
    Q = zeros(num_states, num_actions);
end

num_episodes = 1000;

for episode = 1:num_episodes
    robotPosition = startPosition;
    reached_goal = false;
    
    while ~reached_goal
        stateIndex = sub2ind(mazeSize, robotPosition(1), robotPosition(2)); %TODO: add orientation to state? alsov unique identifier for the maze rather than just the size?
        
        action = choose_action(stateIndex, Q, epsilon);
        nextPosition = perform_action(robotPosition, action);
        reward = calculate_reward(robotPosition, nextPosition, reached_goal);
        nextStateIndex = sub2ind(mazeSize, nextPosition(1), nextPosition(2));
        update_Q(Q, stateIndex, action, reward, nextStateIndex, alpha, gamma);
        
        update_visualization(robotPosition); % TODO: assume this will be needed?
        
        robotPosition = nextPosition;
        
        if isequal(robotPosition, goalPosition)
            reached_goal = true;
    end
    save('Q.mat', 'Q');
end
