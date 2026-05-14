function [dis] = GetLaser(robot_X,robot_Y,robot_t,Eff_robot,K,MODE,draw)
% Coverage MODE
% 1:non-overlapping(break), 2: over-lapping 3: non-overlapping

N=200;
map=zeros(N,N);

robot_N=size(robot_X,2);
grid_resolution=10; % 2 for IGCL, 5 for URL
sensor.r=100;%400/grid_resolution;%80;%80;
degree=60;%120;%60;
sensor.phi=degree;%;degree+1;
sensor.scan=5;%degree*4;
figure(2);
for Iteration=1:1
    %close all;

    % obstacle setting
    map=Map_creator(N);
    % plot lines & compute coverage
    for i=1:robot_N
        position(i,1)=robot_X(i);   
        position(i,2)=robot_Y(i);
        position(i,3)=robot_t(i);   
        robot.x=position(i,1);
        robot.y=position(i,2);
        robot.theta=position(i,3);
        if Eff_robot(i)==1
            [map,dis]= PlotLine(robot,sensor,map,MODE);
        end
    end
    if draw
        figure(2);
        [Ratio,Coverage_rate,NONOL_Coverage_rate,OL_Coverage_rate]=PlotMap(map);hold on;
        Dir_r=4;
        for i=1:robot_N
            if Eff_robot(i)==1
            plot(position(i,1),position(i,2),'go');hold on;     
            line_x=[position(i,1) position(i,1)+Dir_r*cos(position(i,3))];
            line_y=[position(i,2) position(i,2)+Dir_r*sin(position(i,3))];
            line(line_x,line_y,'Color',[0 1 0]);   
            text(position(i,1)+5,position(i,2),num2str(i));
            else
            plot(position(i,1),position(i,2),'rx');hold on;    
            text(position(i,1)+5,position(i,2),num2str(i));        
            end
        end
    end

    for i=1:robot_N
        X_data(1,1+(i-1)*3:3+(i-1)*3)=[position(i,1) position(i,2) position(i,3)];
    end
    X(Iteration,:)=X_data;

    if draw
        Coverage(Iteration,1)=Coverage_rate;
        hold off;
        Eff_N=sum(Eff_robot);hold on;
    end
    %title(['Iteration=',num2str(K),', Coverage=',num2str(Coverage_rate),', Ratio=',num2str(Ratio),', Overlap=',num2str(OL_Coverage_rate), ', Eff_N=',num2str(Eff_N)])
    %fprintf('coverage   =%f\nRatio=%f\noverlap    =%f\n',Coverage_rate,Ratio,OL_Coverage_rate);
    %title([ 'Coverage=',num2str(Coverage_rate)]);
end