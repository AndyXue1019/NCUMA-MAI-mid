clear; clc; close all;

training = true;
training = false;

seed = 42; rng(seed); % for reproducibility
goal.x = 150; goal.y = 250; % goal location
obs.x = 150; obs.y = 150; % obstacle location
robot0.x = 150; robot0.y = 50; robot0.t = 1.57; % robot initial location
% Q-learning weighting
if training
    W = [0; 0; -10; -10];
    % W = zeros(4,1);
else
    % W = [0.484356686828168; 1.48497958937705; -9.19204974083731; -7.18444575584012];
    W = [2.00015549811479; -3.07358973738919; -9.17407270097251; -1.32192115860682];
end
dt = 0.1; % delta t
CoverMODE = 2; M = 1; m = 1; Eff_robot = 1; % GetLaser parameters

if training
    Episode = 500;
    % learning rate
    alpha = 0.01;
    % exploration rate
    epsilon_init = 1.0;
    epsilon_end = 0.05;

    decay_episodes = floor(Episode * 0.9);
    decay_rate_eps = (epsilon_init - epsilon_end) / decay_episodes;
else
    alpha = 0;
    Episode = 1;
end
gamma = 0.9; % discount

history_steps = zeros(Episode, 1);
history_rewards = zeros(Episode, 1);
history_success = zeros(Episode, 1);

for Epi = 1:Episode
    figure(2); clf;
    Terminal = 0;
    robot_t_1.x = robot0.x;
    robot_t_1.y = robot0.y;
    robot_t_1.t = robot0.t;

    robot_data = [robot_t_1.x, robot_t_1.y, robot_t_1.t];
    laser_t_1 = GetLaser(robot_data(1), robot_data(2), robot_data(3), Eff_robot, 1, CoverMODE, ~training);

    a = 3;
    step = 0;
    total_reward = 0;

    if training
        if Epi <= decay_episodes
            epsilon = epsilon_init - decay_rate_eps * Epi;
        else
            epsilon = epsilon_end;
        end
    else
        epsilon = 0;
    end
    disp(['Starting Episode ', num2str(Epi), ', Epsilon: ', num2str(epsilon)]);

    while (Terminal == 0 && step < 1000)
        step = step + 1;

        % motion model
        [robot_t] = motion_model(robot_t_1, a, dt);

        % get reward
        [R, Terminal] = Reward(robot_t, a, goal, obs);

        % get laser data
        robot_data = [robot_t.x, robot_t.y, robot_t.t];
        laser_t = GetLaser(robot_data(1), robot_data(2), robot_data(3), Eff_robot, 1, CoverMODE, ~training);

        % Q-learning update
        [a_next, Wt, J] = Q_learning(a, W, robot_t_1, robot_t, goal, laser_t_1, laser_t, R, Terminal, alpha, gamma, epsilon);

        total_reward = total_reward + R;

        W = Wt;
        a = a_next;
        robot_t_1 = robot_t;
        laser_t_1 = laser_t;

        if ~training
            title(['Episode=', num2str(Epi), ', Step=', num2str(step)]);
            drawnow;
        end
    end

    history_steps(Epi) = step;
    history_rewards(Epi) = total_reward;

    if R == 10 
        history_success(Epi) = 1;
        disp(['[✅ Success] Episode ', num2str(Epi), ...
            ' | Steps: ', num2str(step), ...
            ' | Total Reward: ', num2str(total_reward) ...
            ' | Weights: ', mat2str(W')]);
        fprintf('\n');
    else
        history_success(Epi) = 0;
        disp(['[❌ Failed] Episode ', num2str(Epi), ...
            ' | Steps: ', num2str(step), ...
            ' | Total Reward: ', num2str(total_reward) ...
            ' | Weights: ', mat2str(W')]);
        fprintf('\n');
    end
end

if training
    figure('Name', 'Training Performance', 'Color', 'w');

    % Total Reward per Episode
    subplot(3, 1, 1);
    plot(history_rewards, 'LineWidth', 1.5, 'Color', '#0072BD');
    title('Total Reward per Episode');
    ylabel('Total Reward');
    grid on;

    % Steps per Episode
    subplot(3, 1, 2);
    plot(history_steps, 'LineWidth', 1.5, 'Color', '#D95319');
    title('Steps per Episode');
    ylabel('Steps');
    grid on;

    % Success per Episode
    subplot(3, 1, 3);
    plot(history_success, 'o', 'MarkerFaceColor', '#77AC30', 'MarkerEdgeColor', 'k');
    title('Success (1 = Reached Goal, 0 = Failed)');
    xlabel('Episode');
    ylabel('Status');
    ylim([-0.2 1.2]);
    yticks([0 1]);
    grid on;
end