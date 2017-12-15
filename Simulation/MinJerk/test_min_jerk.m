tf = 0.5;
freq = 100; %hertz

t = 0:1/freq:tf;
traj = min_jerk([0 0 0], [2 2 2], t);

% subplot(3,1,1);
% plot(t, traj(:,1));
% xlabel('t');
% ylabel('x');
% 
% subplot(3,1,2);
% plot(t, traj(:,2));
% xlabel('t');
% ylabel('y');
% 
% subplot(3,1,3);
% plot(t, traj(:,3));
% xlabel('t');
% ylabel('z');