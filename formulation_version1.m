clear all;
%% initialization
age_init = [0,abs(randn),abs(randn),abs(randn)];
%base = [0,0];
poi_coord = [0,0;0,5;3,2;2,0];
no_poi = 4 % base+three sensors
vel = 0.5;

%% initial policy
% 1-base, 2-sensor1, 3-sensor2 .......
%policy = [1,2,1,3,4,1];
policy = [1,2,1,3,4,3,2,1];
% for i = 1:no_poi
%     for j = 1:length(policy)
%         if (policy(j) == i)
%             s(i,j) = 1;
%         else
%             s(i,j) = 0;
%         end
%     end
% end
s = zeros(length(policy),no_poi)
for i = 1:length(policy)
    s(i,policy(i)) = 1;
end
del_t = calculate_time(poi_coord,policy,vel)
% A(:,1) is for base which is 0
A(1,:) = age_init;

for i = 2:length(policy)
    for j = 2:no_poi
        A(i,j) = A(i-1,j) + del_t(i);
        flag1 = 0;
        flag2 = 0;
%         for k = 1:i
%             A(i,j) = A(i,j) - s(i,1)*s(k,j)*A(k,j)
%         end
         for k = i:-1:1
              if(s(i,1) == 1)
                  if(s(k,j) == 1)
                    A(i,j) = A(i,j) - A(k,j);
                    flag2 = flag2 + 1;
                  end
                  if(s(k,1) == 1)
                    flag1 = flag1 + 1;
                  end
              end
              if(flag1 == 2 || flag2 == 1)
                  break
              end
         end
    end
end
   
