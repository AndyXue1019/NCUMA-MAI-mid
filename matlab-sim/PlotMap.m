function [Ratio,Coverage_rate,NONOL_Coverage_rate,OL_Coverage_rate]= PlotMap(map)
N=size(map,1);
pindex=1; % non-overlapping scan index
lpindex=1;% overlapping scan index
obindex=1;% obstacles index

for i=1:N
    for j=1:N
        if map(i,j)==3 % non-overlapping scan
        map_x(pindex)=i;
        map_y(pindex)=j;
        pindex=pindex+1;
        end
        if map(i,j)==2 % obstacles
        ob_x(obindex)=i;
        ob_y(obindex)=j;
        obindex=obindex+1;
        end    
        if map(i,j)==4 % overlapping scan
        lp_x(lpindex)=i;
        lp_y(lpindex)=j;
        lpindex=lpindex+1;
        end    
    end
end
    if obindex>1
    plot(ob_x,ob_y,'k.');hold on; % obstacles
    end
    if pindex>1
    plot(map_x,map_y,'.','Color',[0 0 1]); % non-overlapping scans
    end
    if lpindex>1
    plot(lp_x,lp_y,'.','Color',[0 0 0.7]); % overlapping scans
    end
Coverage_rate=(pindex+lpindex-2)/(N*N-obindex);
NONOL_Coverage_rate=(pindex-1)/(N*N-obindex);
OL_Coverage_rate=(lpindex-1)/(N*N-obindex);
Ratio=Coverage_rate/(1+OL_Coverage_rate);
axis([0 N 0 N]);