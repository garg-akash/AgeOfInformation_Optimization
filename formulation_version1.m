clear all;
%% initialization
age_init = [0,abs(randn),abs(randn),abs(randn)];
%base = [0,0];
poi_coord = [0,0;0,5;3,2;2,0];
no_poi = 4 % base+three sensors
vel = 0.5;

%% initial policy
% 1-base, 2-sensor1, 3-sensor2 .......
policy = [1,2,1,3,4,1];
%policy = [1,2,1,3,4,3,2,1];

s = zeros(length(policy),no_poi)
for i = 1:length(policy)
    s(i,policy(i)) = 1;
end
del_t = calculate_time(poi_coord,policy,vel)
% A(:,1) is for base which is 0
A_uv(1,:) = age_init;
A_b(1,:) = age_init;

for i = 2:length(policy)
    for j = 2:no_poi
        A_uv(i,j) = A_uv(i-1,j) + del_t(i);
        if (s(i,j) == 1)
            A_uv(i,j) = 0;
        end
        A_b(i,j) = (1-s(i,1))*(A_b(i-1,j) + del_t(i)) + s(i,1)*A_uv(i,j);
    end
end


