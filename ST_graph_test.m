clear all 
clc
%% Initialization
path_s_end = 60;
path_l = ones(1,10)*-2;
s = 1:1:path_s_end;
recommended_T = 8;
T = 1:recommended_T;
st = zeros(recommended_T, path_s_end);

%% ST map initialization
for i = 1:recommended_T
    for j = 1:path_s_end
        st(i,j) = s(j);
    end
end

%% reference line definition
ref_s = 1:60;
ref_l = zeros(1,60);
%% Dynamic obstacle definition
dynamic_obs_s = 22;
dynamic_obs_l = 6;
dynamic_obs_s_dot = 0;
dynamic_obs_l_dot = -1;
obs_length = 2;
obs_width = 2;
% dynamic_obs_coordinates = zeros(4,1,1);
% dynamic_obs_coordinates(1) = [dynamic_obs_s_center + obs_width/2, dynamic_obs_l_center - obs_length/2];% left up point
% dynamic_obs_coordinates(2) = [dynamic_obs_s_center + obs_width/2, dynamic_obs_l_center + obs_length/2];% right up point
% dynamic_obs_coordinates(3) = [dynamic_obs_s_center - obs_width/2, dynamic_obs_l_center - obs_length/2];% left down point
% dynamic_obs_coordinates(4) = [dynamic_obs_s_center - obs_width/2, dynamic_obs_l_center + obs_length/2];% right dowm point

% t = 1
%% Dynamic obstacle map
dynamic_obs_s_prediction = zeros(1,recommended_T);
dynamic_obs_l_prediction = zeros(1,recommended_T);
for t = 1:recommended_T
    dynamic_obs_s_prediction(t) = dynamic_obs_s + dynamic_obs_s_dot*t;
    dynamic_obs_l_prediction(t) = dynamic_obs_l + dynamic_obs_l_dot*t;
end
dynamic_obs = zeros(recommended_T, path_s_end);
dynamic_obs_upper_bound = zeros(recommended_T,1);
dynamic_obs_lower_bound = zeros(recommended_T,1);
for t = 1:recommended_T
    if dynamic_obs_l_prediction(t) - obs_length/2 <= 0 && dynamic_obs_l_prediction(t) + obs_length/2 >= 0
        dynamic_obs(t,dynamic_obs_s_prediction(t) - obs_width/2) = nan;
        dynamic_obs(t,dynamic_obs_s_prediction(t)) = nan;
        dynamic_obs(t,dynamic_obs_s_prediction(t) + obs_width/2) = nan;
        dynamic_obs_upper_bound(t) = dynamic_obs_s_prediction(t) + obs_width/2;
        dynamic_obs_lower_bound(t) = dynamic_obs_s_prediction(t) - obs_width/2;
    end
end


%% Cost calculation
% cost_total = cost_pre_minimal_node + cost_obs + cost_edge + cost_goal
% cost_pre_minimal_node = the minimal cost in the last time stamp
% cost_obs = inf if node is Nan in dynamic_obs matrix
%          = 0 if node is 5m (safe distance) far from dynamic_obs
%          = coeff_obs * 1/(s - dynamic_obs_upper_bound)^2 if s is in the upper area of obstacles
%          = coeff_obs * 1/(s - dynamic_obs_lower_bound)^2 if s is in the lower area of obstacles
% cost_edge = cost_distance + cost_velocity
%             cost_distance = 1/2 * coeff_distance * (s - pre_minimal_node_s)^2
%             cost_velocity = 1/2 * coeff_distance * (velocity - pre_velocity)^2
% cost_goal = 1/2 * coeff_cost_goal * (path_s_end - s)^2

% Initialization
plan_start_velocity = 4;
plan_start_s = 0;
time_interval = 1;

% Hyperparameters
coeff_obs = 1000;
coeff_distance = 5;
coeff_velocity = 20;
coeff_goal = 1;

% adaptive ST map based on velocity and acceleration limitation
velocity_max_limitation = 5;
velocity_min_limitation = -5;
acceleration_limitation = 2;
deceleration_limitation = -2;
adaptive_st = ones(recommended_T, path_s_end)*nan;
node_velocity = zeros(recommended_T, path_s_end);
node_acceleration = zeros(recommended_T, path_s_end);
% t = 1
for i = 1:path_s_end
    node_velocity(1,i) = (st(1,i) - plan_start_s)/time_interval;
    node_acceleration(1,i) = node_velocity(1,i) - plan_start_velocity;
    if node_velocity(1,i) <= velocity_max_limitation && node_velocity(1,i) >= velocity_min_limitation ... 
       && node_acceleration(1,i)<= acceleration_limitation && node_acceleration(1,i) >= deceleration_limitation
        adaptive_st(1,i) = st(1,i);
    end
end
% 1 < t < 8     
for t = 2:recommended_T
    for i = 1:path_s_end
        for j = 1:path_s_end
            if isnan(adaptive_st(t-1,j))
                continue;
            else
                node_velocity(t,i) = (st(t,i) - st(t-1,j))/time_interval;
                node_acceleration(t,i) = node_velocity(t,i) - node_velocity(t-1,j);
                if node_velocity(t,i) <= velocity_max_limitation && node_velocity(t,i) >= velocity_min_limitation ... 
               && node_acceleration(t,i)<= acceleration_limitation && node_acceleration(t,i) >= deceleration_limitation
                adaptive_st(t,i) = st(t,i);
                end
            end
        end
    end
end
        
%% Dynamic Programming in average sampling space
node_cost = ones(recommended_T, path_s_end)*inf;
pre_node_index = zeros(recommended_T, path_s_end);
tic
% t = 1
for i = 1:path_s_end
    node_cost(1,i) = CalculateStartCost(i, st, dynamic_obs, dynamic_obs_upper_bound,... 
                     dynamic_obs_lower_bound, plan_start_s, plan_start_velocity, path_s_end, time_interval,...
                     coeff_obs, coeff_distance, coeff_velocity, coeff_goal);
end
% 1 < t < 8 
for t = 2:8
    for i = 1:path_s_end
        if isnan(st(t,i))
            continue;
        end
        for j = 1:path_s_end
            cost_node = CalculateNodeCost(t, i, j, st, dynamic_obs, dynamic_obs_upper_bound,... 
                    dynamic_obs_lower_bound, plan_start_s, path_s_end, time_interval,...
                    coeff_obs, coeff_distance, coeff_velocity, coeff_goal);
            pre_min_cost = node_cost(t-1, j);
            cost_temp = pre_min_cost + cost_node;
            cur_velocity = (st(t,i) - st(t-1,j))/time_interval;
            if cost_temp < node_cost(t, i) && cur_velocity >= -5 && cur_velocity <= 5
                node_cost(t, i) = cost_temp;
                pre_node_index(t, i) = j;
            end
        end
    end
end

index = 0;
min_cost = inf;
for i = 1:path_s_end
    if node_cost(end, i) < min_cost
        min_cost = node_cost(end, i);
        index = i;
    end
end
dp_node_list_row = zeros(recommended_T, 1);
cur_index = index;
for i = 1:recommended_T
    pre_index = pre_node_index(end - i + 1, cur_index);
    dp_node_list_row(recommended_T - i + 1) = cur_index;
    cur_index = pre_index;
end
toc
time_average_st = toc;

%% Dynamic Programming in adaptive sampling space
node_cost = ones(recommended_T, path_s_end)*inf;
pre_node_index = zeros(recommended_T, path_s_end);
tic
% t = 1
for i = 1:path_s_end
    node_cost(1,i) = CalculateStartCost(i, st, dynamic_obs, dynamic_obs_upper_bound,... 
                     dynamic_obs_lower_bound, plan_start_s, plan_start_velocity, path_s_end, time_interval,...
                     coeff_obs, coeff_distance, coeff_velocity, coeff_goal);
end
% 1 < t < 8 
for t = 2:8
    for i = 1:path_s_end
        if isnan(st(t,i))
            continue;
        end
        for j = 1:path_s_end
            cost_node = CalculateNodeCost(t, i, j, st, dynamic_obs, dynamic_obs_upper_bound,... 
                    dynamic_obs_lower_bound, plan_start_s, path_s_end, time_interval,...
                    coeff_obs, coeff_distance, coeff_velocity, coeff_goal);
            pre_min_cost = node_cost(t-1, j);
            cost_temp = pre_min_cost + cost_node;
            cur_velocity = (st(t,i) - st(t-1,j))/time_interval;
            if cost_temp < node_cost(t, i) && cur_velocity >= -5 && cur_velocity <= 5
                node_cost(t, i) = cost_temp;
                pre_node_index(t, i) = j;
            end
        end
    end
end

index = 0;
min_cost = inf;
for i = 1:path_s_end
    if node_cost(end, i) < min_cost
        min_cost = node_cost(end, i);
        index = i;
    end
end
dp_node_list_row = zeros(recommended_T, 1);
cur_index = index;
for i = 1:recommended_T
    pre_index = pre_node_index(end - i + 1, cur_index);
    dp_node_list_row(recommended_T - i + 1) = cur_index;
    cur_index = pre_index;
end
toc
time_adaptive_st = toc;
%% Visualization
figure(1);
scatter(T,st, 'b')
hold on
for i = 1:recommended_T
    for j = 1:path_s_end
        if ~isnan(adaptive_st(i,j))
            scatter(i,j, 'filled', 'g');
        end
    end
end

for i = 1:1:recommended_T
    for j = 1:path_s_end
        if isnan(dynamic_obs(i,j))
            scatter(i, j, 'filled', 'r');
        end
    end
end

line(T, dp_node_list_row);
hold off;
save()
%% CalculateStartCost function
function cost = CalculateStartCost(i, adaptive_st, dynamic_obs, dynamic_obs_upper_bound, dynamic_obs_lower_bound,... 
                     plan_start_s, plan_start_velocity, path_s_end, time_interval,...
                     coeff_obs, coeff_distance, coeff_velocity, coeff_goal)
    
    if isnan(adaptive_st(1,i))
        cost = inf;
    else
        % cost_obs
        safe_distance = 5;
        if isnan(dynamic_obs(1,i))
            cost_obs = inf;
        elseif adaptive_st(1,i) > dynamic_obs_upper_bound(1) + safe_distance || adaptive_st(1,i) < dynamic_obs_lower_bound(1) + safe_distance
            cost_obs = 0;
        elseif adaptive_st(1,i) > dynamic_obs_upper_bound(1)
            cost_obs = coeff_obs*1/(adaptive_st(1,i) - dynamic_obs_upper_bound(1))^2;
        elseif adaptive_st(1,i) < dynamic_obs_lower_bound(1)
            cost_obs = coeff_obs*1/(adaptive_st(1,i) - dynamic_obs_lower_bound(1))^2;
        end

        % cost_edge
        cost_distance = 1/2 * coeff_distance * (adaptive_st(1,i) - 0)^2;
        cur_node_velocity = (adaptive_st(1,i) - plan_start_s)/time_interval;
        cost_velocity = 1/2 * coeff_velocity * (cur_node_velocity - plan_start_velocity)^2;
        cost_edge = cost_distance + cost_velocity;

        % cost_goal
        cost_goal = 1/2 * coeff_goal * (path_s_end - adaptive_st(1,i))^2;

        % total cost
        cost = cost_obs + cost_edge + cost_goal;
    end
end

%% CalculateNodeCost function
function cost = CalculateNodeCost(t, i, j, adaptive_st, dynamic_obs, dynamic_obs_upper_bound, dynamic_obs_lower_bound,... 
                     plan_start_s, path_s_end, time_interval,...
                     coeff_obs, coeff_distance, coeff_velocity, coeff_goal)
    if isnan(adaptive_st(t,i))
         cost = inf;
    else
        
        % cost_obs
        safe_distance = 5;
        if isnan(dynamic_obs(t,i)) || ((adaptive_st(t,i) >= dynamic_obs_lower_bound(t) - 2) && (adaptive_st(t,i) <= dynamic_obs_upper_bound(t) + 2))
            cost_obs = inf;
        elseif adaptive_st(t,i) > dynamic_obs_upper_bound(t) + safe_distance || adaptive_st(t,i) < dynamic_obs_lower_bound(t) + safe_distance
            cost_obs = 0;
        elseif adaptive_st(t,i) > dynamic_obs_upper_bound(t)
            cost_obs = coeff_obs*1/(adaptive_st(t,i) - dynamic_obs_upper_bound(t))^2;
        elseif adaptive_st(t,i) < dynamic_obs_lower_bound(t)
            cost_obs = coeff_obs*1/(adaptive_st(t,i) - dynamic_obs_lower_bound(t))^2;
        end

        % cost_edge
        cost_distance = 1/2 * coeff_distance * (adaptive_st(t,i) - adaptive_st(t-1, j))^2;
        cur_node_velocity = (adaptive_st(t,i) - adaptive_st(t-1,j))/time_interval;
        if isnan(adaptive_st(t-1,j))
            cost_velocity = inf;
        else
            pre_node_velocity = (adaptive_st(t-1,j) - plan_start_s)/((t-1) * time_interval);
            cost_velocity = 1/2 * coeff_velocity * (cur_node_velocity - pre_node_velocity)^2;
         end
        cost_edge = cost_distance + cost_velocity;

        % cost_goal
        cost_goal = 1/2 * coeff_goal * (path_s_end - adaptive_st(t,i))^2;

        % total cost
        cost = cost_obs + cost_edge + cost_goal;
    end
end