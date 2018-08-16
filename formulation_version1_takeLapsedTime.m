clc;
clear all;

%% initialization
%age_init = [0,abs(randn),abs(randn),abs(randn)];
age_init = [0,0,0,0];
%base = [0,0];
poi_coord = [0,0;0,5;3,5;3,0];
poi_no = 4 % base+three sensors

policy = [1,2,1,3,4,1];

vel = 0.5;
lambda = [0;1/3;1/11;1/5];
for i = 1:poi_no
    update_time(i) = 1/lambda(i);
    deadline(i) = 1/lambda(i);
end
p = 1;
current_time(p,1) = 0;
s = zeros(1,poi_no);  % matrix to calculate the age of information
s(p,policy(p)) = 1; % considering slot 1 for initial base position of UAV
% A(:,1) is for base which is 0
A(p,:) = age_init;   % Age matrix


%% Inter poi time calculations
for i=1:poi_no
    for j=1:poi_no
        inter_time(i,j) = sqrt(power(poi_coord(i,1)-poi_coord(j,1),2) + power(poi_coord(i,2)-poi_coord(j,2),2))/vel;
    end
end
future_deadline(1,1) = 0;
future_current_time(1,1) = 0;
for i = 2:(length(policy))
    future_deadline(i,1)  = deadline(policy(i));
    future_current_time(i,1) = (current_time(i-1,1) + inter_time(policy(i-1),policy(i)));
    
    while(future_current_time(i,1) - future_deadline(i,1) >= update_time(policy(i)))
        future_deadline(i,1) = future_deadline(i,1) + update_time(policy(i));
    end
    packet_already_taken(i,1) = future_current_time(i,1) - future_deadline(i,1);
    if (policy(i) == 1)  %For base take dealy due to packet_already_taken as 0
        packet_already_taken(i) = 0;
    end
    
    %% Update slot number,current_time and poi_current
    p = p+1;
    current_time(i,1) = future_current_time(i,1);
    
    %% Update s matrix and age matrix after every run
    
    s = [s;zeros(1,poi_no)];
    s(p,policy(p)) = 1;
    % A(p) = A(p-1) + current_time;
    for j = 2:poi_no
        %A(p,j) = A(p-1,j) + current_time(p);
        A(p,j) = A(p-1,j) + inter_time(policy(p-1),policy(p));
        flag2 = 0;
        flag3 = 0;
        for k = p:-1:1
            if(s(p,1) == 1) % ie., UAV currently at base
                if(s(k,j) == 1) %find which poi other than base recently visited
                    A(p,j) = A(p,j) - A(k,j) + packet_already_taken(k); %reset age for resently collected poi packet
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
    A(p,policy(p)) = A(p,policy(p)) + packet_already_taken(p);
end