function [f] = features(robot, goal, laser, a)
    f = zeros(4,1);

    % (a) f1(S): constant
    f(1) = 1.0;

    % (b) f2(S): distance between the robot and goal
    d = sqrt((robot.x - goal.x)^2 + (robot.y - goal.y)^2);
    f(2) = (d / 300)^2;

    % (c) f3(S): angle between the robot and goal
    angle_to_goal = atan2(goal.y - robot.y, goal.x - robot.x);
    f3 = abs(piangle(angle_to_goal - robot.t)) / pi + 0.1;
    f(3) = f3;

    % (d) f4(S,a): laser data (dis) from direction a1 to a5
    dis = laser(a);
    if dis <= 70
        f(4) = f3;
    else
        f(4) = -f3;
    end
end