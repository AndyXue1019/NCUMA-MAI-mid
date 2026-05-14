function [R, Terminal] = Reward(robot, a, goal, obs)
    Terminal = 0;
    R = -0.05;

    % Obstacle x: 140~160, y: 145~155
    % D_RO_x = max([140 - robot.x, 0, robot.x - 160]);
    % D_RO_y = max([145 - robot.y, 0, robot.y - 155]);
    % D_RO = sqrt(D_RO_x^2 + D_RO_y^2);
    D_RO = sqrt((robot.x - obs.x)^2 + (robot.y - obs.y)^2);

    D_RG = sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2);

    D_RB_x = min(robot.x, 300 - robot.x);
    D_RB_y = min(robot.y, 300 - robot.y);
    D_RB = min(D_RB_x, D_RB_y);

    if D_RO < 5
        R = -10;
        Terminal = 1;
    elseif D_RB < 5
        R = -10;
        Terminal = 1;
    elseif D_RG < 10
        R = 10;
        Terminal = 1;
    end
end