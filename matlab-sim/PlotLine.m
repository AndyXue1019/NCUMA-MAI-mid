function [map,dis]= PlotLine(robot,sensor,map,MODE)

N=size(map,1);
dis=100*ones(5,1);
% plot a line
 % add angle resolution?
for dtheta=1:sensor.scan
    theta=-0.0174*36+(0.0174*sensor.phi/sensor.scan)*dtheta;

    r=sensor.r;
    resolution=sensor.r;
    dx=r*cos(robot.theta+theta)/resolution;
    dy=r*sin(robot.theta+theta)/resolution;
    for i=1:resolution
    di=fix(robot.x+dx*i);
    dj=fix(robot.y+dy*i);
        if ((di>0) && (di<N)) && ((dj>0) && (dj<N)) && (map(di,dj)==2) % obstacles inside
        dis(dtheta)=i;
        break;
        end
        if ((di>0) && (di<N)) && ((dj>0) && (dj<N))
            if map(di,dj)==3
                if MODE==1
                    if MODE==1
                    break;
                    elseif MODE==3
                    end
                elseif MODE==2
                map(di,dj)=4; % overlapping cells
                end
            elseif map(di,dj)==0
            map(di,dj)=1; % scanned cell
            end
        end
    end
end

for i=1:N
    for j=1:N
       if map(i,j)==1
       map(i,j)=3;  
       end
    end
end

%dis
