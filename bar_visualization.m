figure(1);
bar(time_dp_average_sampling_data);
ylim([0 0.01]);
hold on;
% subplot(2,1,2) 
bar(time_dp_adaptive_sampling);
ylabel('\it running time(seconds)')
xlabel('\it times')
legend('running time in average sampling','running time in adaptive sampling');

% figure(1);
% bar(minimal_cost_average_sampling);
% figure(2);
% ylim([0 1200]);
% bar(minimal_cost_adaptive_sampling(2:end));


% figure(1);
% bar(time_average);
% figure(2);
% 
% bar(time_adaptive);
% ylim([0 0.035]);