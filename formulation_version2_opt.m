clc;
clear all;

%% initialization
age_init = [0,abs(randn),abs(randn),abs(randn)];
%base = [0,0];
poi_coord = [0,0;0,5;3,5;3,0];
poi_no = 4 % base+three sensors
poi_start = 1 %Base is the start node
poi_current = poi_start; %For inital run start node is the current node

vel = 0.5;
lambda = [0;1/3;1/7;1/11]; 
for i = 1:poi_no
    update_time(i) = 1/lambda(i);
    deadline(i) = 1/lambda(i);
end
p = 1; % keep track of slot number or no of times UAV hops
current_time(p) = 0;
s = zeros(1,poi_no);  % matrix to calculate the age of information
s(p,poi_current) = 1; % considering slot 1 for initial base position of UAV 
% A(:,1) is for base which is 0
A(p,:) = age_init;   % Age matrix


%% Inter poi time calculations
for i=1:poi_no
    for j=1:poi_no
        inter_time(i,j) = sqrt(power(poi_coord(i,1)-poi_coord(j,1),2) + power(poi_coord(i,2)-poi_coord(j,2),2))/vel;
    end
end

for r = 1:2
    % for i = 1:poi_no
    %     future_deadline(i,:)  = deadline;
    % end
    future_deadline(p,:)  = deadline;
    %for i = 1:poi_no
    for j = 1:poi_no
        future_current_time(p,j) = (current_time(p) + inter_time(poi_current,j));
        
        while(future_current_time(p,j) > future_deadline(p,j))
            future_deadline(p,j) = future_deadline(p,j) + update_time(j);
        end
        if j==poi_current % for poi_current, sample has been collected therefore, consider next sample
            future_deadline(p,poi_current) = future_deadline(p,poi_current) + update_time(poi_current);
        end
        wait_time(p,j) = future_deadline(p,j) - future_current_time(p,j);
        cost1(p,j) = inter_time(poi_current,j) + wait_time(p,j);
    end
    %end
    % [dead,dead_index] = min(cost1);
    % dead_current = poi_current;
    [dead,dead_next] = min(cost1(p,:));
    
    %% Update slot number,current_time and poi_current
    p = p+1;
    current_time(p) = current_time(p-1) + cost1(p-1,dead_next); % current time should be increased not just by
    % inter_time but also by waiting time ie, by cost1
    poi_current = dead_next;
    
    %% Update s matrix and age matrix after every run
    
    s = [s;zeros(1,poi_no)];
    s(p,poi_current) = 1;
    % A(p) = A(p-1) + current_time;
    for j = 2:poi_no
        %A(p,j) = A(p-1,j) + current_time(p);
        A(p,j) = A(p-1,j) + dead;
        flag2 = 0;
        flag3 = 0;
        for k = p:-1:1
            if(s(p,1) == 1) % ie., currently at base
                if(s(k,j) == 1) %find which poi other than base recently visited
                    A(p,j) = A(p,j) - A(k,j); %reset age for resently collected poi
                    flag3 = flag3 + 1;
                end
                if(s(k,1) == 1)
                    flag2 = flag2 + 1;
                end
            end
            if(flag2 == 2 || flag3 == 1)
                break
            end
        end
    end
end