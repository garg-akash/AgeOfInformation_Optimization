function [t] = calculate_time(poi_coord, policy, vel)
t(1,1) = 0;
for i = 2:length(policy)
    t(i,1) = sqrt(power(poi_coord(policy(i),1)-poi_coord(policy(i-1),1),2) + power(poi_coord(policy(i),2)-poi_coord(policy(i-1),2),2))/vel;
end
end
