%% Data Analysis of String Optimization
clc; clear; close all;
currentDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

global counter
counter = 1;

load('trialSineTrajectories.mat');

%% Evaluate Objective Function
% Min Catch Time
for i=1:length(trial)
    trial(i).cost = 5e2*trapz(trial(i).u_nom(1,:)) + 0.5e13*trapz(trial(i).u_nom(3,:)) + 1*max(abs(trial(i).x_nom(5,:))) - 5*trial(i).l;
end

for i=1:length(trial)
   costs(i) = trial(i).cost;
   inputs(i) = sum(trial(i).u_nom(1,:));
   lengths(i) = trial(i).l;
end
costs = costs - mean(costs); %normalize
t= trial(1).t;

% %% Optimization Plots
% figure(1);
% hold on;
% [best, ind] = min(costs);
% plot(lengths(1:end),costs(1:end),'LineWidth',3);
% scatter(lengths(ind),costs(ind),500,'g.');
% hold off;
% xlabel('String Length (m)','FontSize',14);
% ylabel('Objective Function Value','FontSize',14);
% title(['Optimal String Length = ',num2str(lengths(ind)),'m'],'FontSize',14);
% ylim([-0.5 1.5]);
% 
% %% Get Frequency
% slopes = diff(trial(ind).x_nom(2,:));
% peaks = [];
% for i=1:length(slopes)-1
%     if slopes(i) > 0 && slopes(i+1) < 0
%         peaks = [peaks; [trial(ind).x_nom(2,i+1) trial(ind).t(i+1)]];
%     end
% end
% g = 9.81;
% periodPend = 2*pi*sqrt(lengths(ind)/g);
% period = min(diff(peaks(:,2)));
% frequency = 1/period;
% frequency = frequency*2*pi;
% 
% figure(2)
% hold on;
% plot(trial(ind).t,smooth(trial(ind).x_nom(2,:),1),'LineWidth',3,'Color','blue');
% scatter(peaks(:,2),peaks(:,1),50,'ro');
% hold off;
% xlabel('Time (sec)','FontSize',14);
% ylabel('Quad X Location','FontSize',14);
% title(['Swingup Oscillation Frequency = ',num2str(round(frequency,2)),'m'],'FontSize',14);

%% Optimal String Length Trajectory
% [min, ind] = min(costs);
% x_nom = trial(ind).x_nom;
% param.l = trial(ind).l;
% Anim.speed = 1;
% Anim.plotFunc = @(x_nom)(drawRobot(x_nom,param));
% Anim.verbose = true;
% animate(t,x_nom,Anim);

%% Animate all da trajectories
Anim.speed = 0.025; %0.025
Anim.plotFunc = @(x_noms,lengths)(drawRobots(x_noms,lengths));
Anim.verbose = true;
animateMultiple(t,trial,Anim);