function [robot_t] = motion_model(robot_t_1, a, dt)
    VW.v = 15;

    w_deg = [-30, -15, 0, 15, 30];
    VW.w = w_deg(a) * pi / 180;

    %%motion model
    robot_t.x = robot_t_1.x + VW.v * cos(robot_t_1.t) * dt;
    robot_t.y = robot_t_1.y + VW.v * sin(robot_t_1.t) * dt;
    robot_t.t = piangle(robot_t_1.t + VW.w * dt);
end