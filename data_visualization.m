% Data visualizaiton
figure;
% 
% Time = 1:252;
% subplot(2,1,1)  % 两行一列子图的第一个子图
% plot(Time, time_dp_average_sampling_data,'k','LineWidth',1)  
% % Hold on;
% % Plot(time, torqueRl, 'k:')
% %Legend('FL','RL');  % 给出线型变量定义
% %xlabel('t/s')
% ylabel('\itdelta (deg)')
% grid on

% subplot(2,1,2) 
plot(Time, time_dp_average_sampling_data,'k','LineWidth',1)
ylabel('\ittime(milliseconds)')
xlabel('\ittimes')
hold on
plot(Time, time_dp_adaptive_sampling,'k--','LineWidth',1)
legend('average_sampling_running_time)','adaptive_sampling_running_time');
grid on

% subplot(3,1,3)  
% plot(SlipFL.Time, SlipFL.Data,'k','LineWidth',1)
% ylabel('\itSlip(-)')
% xlabel('\ittime(s)')