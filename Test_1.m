%%This code calculates the age at UAV,age at Base considering wait as well as delay due to update time, 
%%of each sensor and time to travel b/w pois. It visits poi with least cost
%%Problem: Stuck at sensor 1. Change cost function, take age in cost
clc;
clear all;

%% initialization
age_init = [0,0,0];
%base = [0,0];
a = 6;
poi_coord = [a/2,(sqrt(3)/2)*a;0,0;a,0];
poi_no = 3 % base+three sensors
poi_start = 1 %Base is the start node
poi_current = poi_start; %For inital run start node is the current node

vel = 1;
lambda = [0;1/4;1/6]; 
for i = 1:poi_no
    update_time(i) = 1/lambda(i);
    deadline(i) = 1/lambda(i);
end
p = 1; % keep track of slot number or no of times UAV hops
current_time(p) = 0;
s = zeros(1,poi_no);  % matrix to calculate the age of information
s(p,poi_current) = 1; % considering slot 1 for initial base position of UAV 
% A(:,1) is for base which is 0
A_uv(1,:) = age_init;
A_b(1,:) = age_init;

%% Inter poi time calculations
for i=1:poi_no
    for j=1:poi_no
        inter_time(i,j) = sqrt(power(poi_coord(i,1)-poi_coord(j,1),2) + power(poi_coord(i,2)-poi_coord(j,2),2))/vel;
    end
end

for r = 1:3
    future_deadline(p,:)  = deadline;
    %for i = 1:poi_no
    for j = 1:poi_no
        future_current_time(p,j) = (current_time(p) + inter_time(poi_current,j));
        delay_time(p,j) = mod(future_current_time(p,j),update_time(j))
        if (p>1)
            if ((future_current_time(p,j) == future_current_time(p-1,j)) && delay_time(p,j)==0)
                future_current_time(p,j) = future_current_time(p,j) + update_time(j);
                wait_time(p,j) = update_time(j);
            else
                wait_time(p,j) = 0;
            end
            cost1(p,j) = inter_time(poi_current,j) + delay_time(p,j) + wait_time(p,j);
        else
            cost1(p,j) = inter_time(poi_current,j) + delay_time(p,j);
        end
        
    end
    %end
    % [dead,dead_index] = min(cost1);
    % dead_current = poi_current;
    [dead,dead_next] = min(cost1(p,:));
    
    %% Update slot number,current_time and poi_current
    p = p+1;
    current_time(p) = current_time(p-1) + cost1(p-1,dead_next); % current time should be increased not just by
    % inter_time but also by waiting time ie, by cost1
    poi_previous = poi_current;
    poi_current = dead_next;
    
    %% Update s matrix and age matrix after every run
    
    s = [s;zeros(1,poi_no)];
    s(p,poi_current) = 1;
    % A(p) = A(p-1) + current_time;
    for j = 2:poi_no
        %A(p,j) = A(p-1,j) + current_time(p);
        A_uv(p,j) = A_uv(p-1,j) + dead;
        if (s(p,j) == 1)
            A_uv(p,j) = 0;
        end
        A_peak(p,j) = s(p,1)*(A_b(p-1,j) + inter_time(poi_previous,poi_current)); 
        A_b(p,j) = (1-s(p,1))*(A_b(p-1,j) + dead) + s(p,1)*A_uv(p,j);   
        
    end
end