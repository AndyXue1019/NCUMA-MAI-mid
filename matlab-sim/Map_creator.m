function [map]=Map_creator(N)
map=load('map1.mat');map=map.map;

for i=1:300
    for j=1:300
        map(i,j)=0;
    end
end

for i=300
    for j=1:300
        map(i,j)=2;
    end
end
for i=1
    for j=1:300
        map(i,j)=2;
    end
end
for i=1:300
    for j=1
        map(i,j)=2;
    end
end
for i=1:300
    for j=300
        map(i,j)=2;
    end
end

for i=140:160
    for j=145:155
        map(i,j)=2;
    end
end
