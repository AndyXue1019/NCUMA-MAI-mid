function [output] = piangle(theta)
    if theta > pi
        output = theta - 2*pi;
    elseif theta < -pi
        output = theta + 2*pi;
    else
        output = theta;
    end
end