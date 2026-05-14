function [opt_a, Wt, J] = Q_learning(a_prev, W, robot_prev, robot_t, goal, laser_prev, laser_t, R, Terminal, alpha, gamma, epsilon)
    f_prev = features(robot_prev, goal, laser_prev, a_prev);
    Q_prev = W' * f_prev;

    if Terminal == 1
        max_Q_next = 0; % Terminal state, no future reward
    else
        Q_next = zeros(5,1);
        for act = 1:5
            sim_robot = motion_model(robot_t, act, 0.1);
            f_next = features(sim_robot, goal, laser_t, act);
            Q_next(act) = W' * f_next;
        end
        max_Q_next = max(Q_next);
    end

    J = R + gamma * max_Q_next - Q_prev;
    Wt = W + alpha * J * f_prev;

    if Terminal == 1
        opt_a = 3;
    else
        if rand < epsilon
            opt_a = randi([1 5]);
        else
            Q_next_updated = zeros(5, 1);
            for act = 1:5
                sim_robot = motion_model(robot_t, act, 0.1);
                f_next = features(sim_robot, goal, laser_t, act);
                Q_next_updated(act) = Wt' * f_next;
            end
            [~, opt_a] = max(Q_next_updated);
        end
    end
end