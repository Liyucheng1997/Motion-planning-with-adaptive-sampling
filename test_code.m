clear all
clc
%% hyperparameters definition
col = 20;
row = 11;
plan_start_s = 0;
sample_s = 5;
sample_l = 1;
coff_ref = 10;
coff_obs = 20;
coff_bound = 10;
big_number = 1000;

% reference_line
ref_s = 5:sample_s:100;
ref_l = zeros(1,col);

%%  obstacle information
static_obs_s = 5:sample_s:100;
static_obs_l = zeros(2,col);
for i=1:2
    for j = 1:col
        static_obs_l(i,j) = nan;
    end
end
% obstacle 1 ((50,1) -> (55,-1))
static_obs_l(1,10) = 1;
static_obs_l(2,10) = -1;
static_obs_l(1,11) = 1;
static_obs_l(2,11) = -1;

% obstacle 2 ((83,2) -> (88, -1))
static_obs_l(1,17) = 2;
static_obs_l(2,17) = -1;



%% boundary information
lmax = eye(1,col);
lmin = eye(1,col);

for i=1:col
    lmax(i) = 5;
    lmin(i) = -5;
end

%% average sampling
cur_node_s = zeros(col,1);
cur_node_l = zeros(col, row);

for i = 1:col
    for j = 1:row
        cur_node_s(i) = plan_start_s + i*sample_s;
        cur_node_l(i,j) = ((row + 1)/2 - j)*sample_l;
    end
end
%% Adaptive sampling
Uref = zeros(col,row);
Uobs = zeros(col,row);
Ubound = zeros(col,row);
Uapf = zeros(col,row);
% cost calculation
for i = 1:col
    for j = 1:row
        % reference line attration calculation
        Uref(i,j) = 1/2*coff_ref*(cur_node_l(i,j) - ref_l(i))^2;
        
        % static obstacle repulsion calculation
        if isnan(static_obs_l(1,i))
            Uobs(i,j) = 0;
        elseif cur_node_l(i,j) > static_obs_l(1,i)
                Uobs(i,j) = 1/2*coff_obs*(1/(cur_node_l(i,j) - (static_obs_l(1,i)))^2);
        elseif cur_node_l(i,j) < static_obs_l(2,i)
                Uobs(i,j) = 1/2*coff_obs*(1/(cur_node_l(i,j) - (static_obs_l(2,i)))^2);
        else
                Uobs(i,j) = big_number;
        end
        
        % road boundary repulsion calculation
        if (cur_node_l(i,j) >= lmax(i)-1) || (cur_node_l(i,j) <= lmin(i)+1)
            Ubound(i,j) = big_number/2;
        else
            Ubound(i,j) = 1/2*coff_bound*((1/(cur_node_l(i,j) - (lmax(i)-1))^2) + (1/(cur_node_l(i,j) - (lmin(i)+1))^2));
        end
        % total artificial potential field
        Uapf(i,j) = Uref(i,j) + Uobs(i,j) + Ubound(i,j);
    end
end

% choose 5 points with minimal cost
k = 5;
adaptive_sampling_output = zeros(col, k);
for i = 1:col
    for j = 1:k
        [min_value,index] = min(Uapf(i,:));
        adaptive_sampling_output(i,j) =((row + 1)/2 - index)*sample_l;
        Uapf(i,index) = 2 * big_number;
    end
end
%% visulization
rectangle('Position', [50,-1,5,2])
hold on
rectangle('Position', [83,-1,5,3])
scatter(cur_node_s, cur_node_l, 'b')
% line([5 100], [-5, -5], 'color', 'r') % lower boundary line
% line([5 100], [5, 5], 'color', 'r') % upper boundary line
% line([5 100], [0, 0], 'color', 'g') % reference line
% scatter(cur_node_s, adaptive_sampling_output, 'r')  