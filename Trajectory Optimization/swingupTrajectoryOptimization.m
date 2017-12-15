%% Swing-Up Trajectory Optimization %%
clc; clear; close all;
currentDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(currentDir, 'OptimTraj')));

%% Parameters
param = getParameters();
param.T = 8;
param.numSegments = 30; %30
param.TFlex = 0;
param.xFlex = zeros(16,1);

%% Initial Conditions
x0 = zeros(16,1);
x0(1) = 0;
x0(2) = 0;
x0(3) = 0.5;
x0(13) = pi; %ball starts at rest

%Goal State
xF = x0;
xF(1) = 0;
xF(2) = 0;
xF(3) = 0.5;
xF(13) = 0; %ball is swung up

%% Function Handles
problem.func.dynamics =  @(t,x,u)(dynamics(t,x,u)); %dynamics
problem.func.pathObj = @(t,x,u)(objective(x,u)); %objective function

%% Bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = param.T - param.TFlex;
problem.bounds.finalTime.upp = param.T + param.TFlex;

% State: (inertial reference frame)
%   1 = x
%   2 = y
%   3 = z
%   4 = dx
%   5 = dy
%   6 = dz
%   7 = roll
%   8 = pitch
%   9 = yaw
%   10 = droll
%   11 = dpitch
%   12 = dyaw
% State: (quad reference frame)
%   13 = phi
%   14 = theta
%   15 = dphi
%   16 = dtheta

problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
problem.bounds.finalState.low = xF - param.xFlex;
problem.bounds.finalState.upp = xF + param.xFlex;

problem.bounds.state.low = [-0.5 -0.5 0.45 -10 -10 -10 -pi/4 -pi/4  -pi/4 -10 -10 -10 -2*pi -10 -10 -10]';
problem.bounds.state.upp = [ 0.5  0.5 0.55  10  10  10  pi/4  pi/4   pi/4  10  10  10  2*pi  10  10  10]';

problem.bounds.control.low = [0   -0.0001 -0.0001 -0.0001]';
problem.bounds.control.upp = [0.4  0.0001  0.0001  0.0001]';

%% Initial Guess
u_guess = zeros(4,2);
u_guess(1,:) = [param.m*param.g param.m*param.g];
problem.guess.time = [0,param.T];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = u_guess;

%% Options
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...
    'TolFun',1e-3,...
    'MaxFunEvals',1e6);
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = param.numSegments;
problem.options(1).nlpOpt.GradConstr = 'off';
problem.options(1).nlpOpt.GradObj = 'off';
problem.options(1).nlpOpt.DerivativeCheck = 'off';

%% Trajectory Optimization
tic
solution = optimTraj(problem);
toc

t = solution(end).grid.time;
x_nom = solution(end).grid.state(1:16,:);
u_nom = solution(end).grid.control;
param.Tspan = t;

% %% TV-LQR Feedback Controller
% % %Solve Ricatti Equation Backwards in Time
% % Q = eye(16);
% % Q(1,1) = 0.1;
% % Q(2,2) = 0.1;
% % Q(3,3) = 1e4;
% % Q(4,4) = 0.01;
% % Q(5,5) = 0.01;
% % Q(6, 6) = 1e3;
% % Q(7, 7) = 1e2;
% % Q(8, 8) = 1e2;
% % Q(9, 9) = 1e2;
% % Q(10, 10) = 1e1;
% % Q(11, 11) = 1e1;
% % Q(12, 12) = 1e1;
% % Q(13,13) = 0.1;
% % Q(14,14) = 0.1;
% % Q(15,15) = 0.01;
% % Q(16,16) = 0.01;
% % 
% % R = eye(4);
% % R(1, 1) = 1e3;
% % R(2, 2) = 1e5;
% % R(3, 3) = 1e5;
% % R(4, 4) = 1e5;
% 
% Q = eye(16);
% Q(1,1) = 1e-1;
% Q(2,2) = 1e-1;
% Q(3,3) = 1e4;
% Q(4,4) = 1e-2;
% Q(5,5) = 1e-2;
% Q(6, 6) = 1e3;
% Q(7, 7) = 1e2;
% Q(8, 8) = 1e2;
% Q(9, 9) = 1e2;
% Q(10, 10) = 1e1;
% Q(11, 11) = 1e1;
% Q(12, 12) = 1e1;
% Q(13,13) = 1e-4;
% Q(14,14) = 1e-5;
% Q(15,15) = 1e-5;
% Q(16,16) = 1e-5;
% 
% R = eye(4);
% R(1, 1) = 1e3;
% R(2, 2) = 1e5;
% R(3, 3) = 1e5;
% R(4, 4) = 1e5;
% 
% Qf = Q;
% 
% P = zeros(16,16,length(param.Tspan));
% P(:,:,length(param.Tspan)) = Qf;
% 
% % Find K DT
% for i = length(param.Tspan):-1:2
%    dt = diff(param.Tspan(1:2));
%    x_lin = x_nom(:,i);
%    u_lin = u_nom(:,i);
%    [A,B] = linearize(x_lin,u_lin);
%    C = eye(16);
%    D = 0;
%    sys = ss(A,B,C,D);
%    sys_dt = c2d(sys,dt);
%    [A_dt,B_dt,~,~] = ssdata(sys_dt);
%    P(:,:,i-1) = A_dt'*P(:,:,i)*A_dt - A_dt'*P(:,:,i)*B_dt*inv(B_dt'*P(:,:,i)*B_dt+R)*B_dt'*P(:,:,i)*A_dt + Q;
%    K(i,:,:) = inv(R+B_dt'*P(:,:,i)*B_dt)*B_dt'*P(:,:,i)*A_dt;
% end 
% 
% % %Find K CT
% % P = Qf;
% % Pvec = reshape(P,length(P)*length(P),1);
% % Tspan = fliplr(param.Tspan);
% % [t,Pvec] = ode45(@(t,Pvec)ricatti(t,Pvec,Q,R,x_nom,u_nom,param),Tspan,Pvec);
% % for i = 1:length(t)   
% %     P(:,:,i) = reshape(Pvec(length(t)-i+1,:),length(P),length(P));
% % end
% % for i = 1:length(param.Tspan)
% %     t = param.Tspan(i);
% %     [x_lin,u_lin] = interpolateTrajectory(x_nom,u_nom,t,param);
% %     [~,B] = linearize(x_lin,u_lin);
% %     K(i,:,:) = R\B'*P(:,:,i);
% % end
% 
% param.K = K;
% % param.K = zeros(param.numSegments,4,16);
% 
% %% Simulate System
% [t,x] = ode45(@(t,x)dynamicsSimTrajectory(t,x,x_nom,u_nom,param),param.Tspan,x0);
% t = t';
% x = x';

%% Animate Solution
global counter
counter = 1;

Anim.speed = 0.1;
Anim.plotFunc = @(x_nom)(drawRobot(x_nom));
Anim.verbose = true;
animate(t,x_nom,Anim);

% %% Get Frequency
% slopes = diff(x_nom(2,:));
% peaks = [];
% for i=1:length(slopes)-1
%     if slopes(i) > 0 && slopes(i+1) < 0
%         peaks = [peaks; [x_nom(2,i+1) t(i+1)]];
%     end
% end
% 
% periodPend = 2*pi*sqrt(param.l/param.g);
% period = min(diff(peaks(:,2)));
% frequency = 1/period;
% frequency = frequency*2*pi;
% 
% figure(2)
% hold on;
% plot(t,smooth(x_nom(2,:),1),'LineWidth',3,'Color','blue');
% scatter(peaks(1:end-1,2),peaks(1:end-1,1),50,'ro');
% hold off;
% xlabel('Time (sec)','FontSize',14);
% ylabel('Quad X Location (m)','FontSize',14);
% title(['Swingup Oscillation Frequency = ',num2str(round(frequency,2)),' rad/s'],'FontSize',14);

% %% Build Trajectory for Python
% dt = diff(t(1:2));
% x = x_nom(1,:);
% y = x_nom(2,:);
% z = x_nom(3,:)-0.5;
% fprintf('\nsequence = [\n');
% for i=1:length(x)
%     fprintf('\t(%0.6f, %0.6f, %0.6f, %0.6f), \n',x(i),y(i),z(i),dt);
% end
% fprintf(']\n');