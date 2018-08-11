clear all;

%% initialization
age_init = [0,abs(randn),abs(randn),abs(randn)];
%base = [0,0];
poi_coord = [0,0;0,5;3,5;3,0];
poi_no = 4 % base+three sensors
poi_start = 1 %Base is the start node
poi_current = poi_start; %For inital run start node is the current node
current_time = 0;
vel = 0.5;
lambda = [0;1/3;1/7;1/11]; 
for i = 1:poi_no
    update_time(i) = 1/lambda(i);
    deadline(i) = 1/lambda(i);
end
p = 1; % keep track of slot number or no of times UAV hops
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
%for i = 1:2
flag1 = 1;
while(flag1)                 % If all entries of deadline becomes nan it choses the first entry as nan
    [dead,dead_index] = min(deadline); % get least of deadline
    %dead_index = 1 only when all entries become nan, therefore shift to
    %any other sensor
    if (dead_index == 1)
        dead_index = dead_index + 1;
    end
    if (dead < inter_time(poi_current,dead_index))
        deadline(dead_index) = nan;
        disp('Updated deadline is')
        deadline
        flag1 = 1;
    else                     % i.e., I reached at poi before the deadline
        flag1 = 0
        current_time = current_time + inter_time(poi_current,dead_index)
        poi_current = dead_index
        temp_time = update_time(dead_index)
        while(temp_time<current_time)
            temp_time = temp_time + update_time(dead_index); % waiting till multiple of update_time is reached 
        end
        current_time = temp_time   % data collected when multiple of update_time is reached 
        deadline = update_time; % restore deadline vector (so as to remove any nan values)
        for j=1:length(deadline)
            while (current_time >= deadline(j))
                deadline(j) = deadline(j) + update_time(j);
            end
        end
        deadline
    end
end

%% Update slot number,s matrix and age matrix after every run
p = p+1;
s = [s;zeros(1,poi_no)];
s(p,poi_current) = 1;
% A(p) = A(p-1) + current_time;
for j = 2:poi_no
    A(p,j) = A(p-1,j) + current_time;
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
        if(flag1 == 2 || flag3 == 1)
            break
        end
    end
end
%end

    
