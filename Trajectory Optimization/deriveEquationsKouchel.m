clear; close all; clc; 
%% Derive Quadrotor with Inverted Pendulum Dynamics
disp('Defining symbolics...');

syms dp drotm x y z dx dy dz ddx ddy ddz phi theta psi dphi dtheta dpsi ddphi ddtheta ddpsi f tauphi tautheta taupsi xBall yBall zBall dxBall dyBall dzBall ddxBall ddyBall ddzBall 'real'
syms X(t) Y(t) Z(t) XBall(t) YBall(t) ZBall(t)
dX = diff(X,t);
dY = diff(Y,t);
dZ = diff(Z,t);
ddX = diff(dX,t);
ddY = diff(dY,t);
ddZ = diff(dZ,t);
dXBall = diff(XBall,t);
dYBall = diff(YBall,t);
dZBall = diff(ZBall,t);
ddXBall = diff(dXBall,t);
ddYBall = diff(dYBall,t);
ddZBall = diff(dZBall,t);

%% Parameters
param = getParameters();
Ixx = param.Ixx;
Iyy = param.Iyy;
Izz = param.Izz;
mQ = param.m;
g = param.g;
mL = param.mL;
z_offset = param.z_offset;
l = param.l;
J = diag([Ixx Iyy Izz]);

%% Time Derivatives
disp('Evaluating time Derivatives...');

XBar = XBall - X;
YBar = YBall - Y;
ZBar = ZBall - Z;
r = sqrt(XBar^2 + YBar^2 +ZBar^2);
phiBall = atan(YBar/ZBar);
thetaBall = acos(ZBar/r);
dphiBall = diff(phiBall);
dthetaBall = diff(thetaBall);
ddphiBall = diff(dphiBall);
ddthetaBall = diff(dthetaBall);
phiBall = subs(phiBall,{XBall,YBall,ZBall,X,Y,Z},{xBall,yBall,zBall,x,y,z});
thetaBall = subs(thetaBall,{XBall,YBall,ZBall,X,Y,Z},{xBall,yBall,zBall,x,y,z});
dphiBall = subs(dphiBall,{XBall,dXBall,YBall,dYBall,ZBall,dZBall,X,dX,Y,dY,Z,dZ},{xBall,dxBall,yBall,dyBall,zBall,dzBall,x,dx,y,dy,z,dz});
dthetaBall = subs(dthetaBall,{XBall,dXBall,YBall,dYBall,ZBall,dZBall,X,dX,Y,dY,Z,dZ},{xBall,dxBall,yBall,dyBall,zBall,dzBall,x,dx,y,dy,z,dz});
w = [((dy - dyBall)/(z - zBall) - ((dz - dzBall)*(y - yBall))/(z - zBall)^2)/((y - yBall)^2/(z - zBall)^2 + 1);...
 ((dz - dzBall)/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(1/2) - ((z - zBall)*(2*(dx - dxBall)*(x - xBall) + 2*(dy - dyBall)*(y - yBall) + 2*(dz - dzBall)*(z - zBall)))/(2*((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(3/2)))/(1 - (z - zBall)^2/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2))^(1/2);...
                                                                                                                                                                                                                                                                                                                   0];
ddphiBall = subs(ddphiBall,{XBall,dXBall,ddXBall,YBall,dYBall,ddYBall,ZBall,dZBall,ddZBall,X,dX,ddX,Y,dY,ddY,Z,dZ,ddZ},{xBall,dxBall,ddxBall,yBall,dyBall,ddyBall,zBall,dzBall,ddzBall,x,dx,ddx,y,dy,ddy,z,dz,ddz});
ddthetaBall = subs(ddthetaBall,{XBall,dXBall,ddXBall,YBall,dYBall,ddYBall,ZBall,dZBall,ddZBall,X,dX,ddX,Y,dY,ddY,Z,dZ,ddZ},{xBall,dxBall,ddxBall,yBall,dyBall,ddyBall,zBall,dzBall,ddzBall,x,dx,ddx,y,dy,ddy,z,dz,ddz});
dw =    [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              ((ddy - ddyBall)/(z - zBall) + (2*(dz - dzBall)^2*(y - yBall))/(z - zBall)^3 - (2*(dy - dyBall)*(dz - dzBall))/(z - zBall)^2 - ((ddz - ddzBall)*(y - yBall))/(z - zBall)^2)/((y - yBall)^2/(z - zBall)^2 + 1) + (((dy - dyBall)/(z - zBall) - ((dz - dzBall)*(y - yBall))/(z - zBall)^2)*((2*(dz - dzBall)*(y - yBall)^2)/(z - zBall)^3 - (2*(dy - dyBall)*(y - yBall))/(z - zBall)^2))/((y - yBall)^2/(z - zBall)^2 + 1)^2;...
 ((ddz - ddzBall)/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(1/2) - ((dz - dzBall)*(2*(dx - dxBall)*(x - xBall) + 2*(dy - dyBall)*(y - yBall) + 2*(dz - dzBall)*(z - zBall)))/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(3/2) + (3*(z - zBall)*(2*(dx - dxBall)*(x - xBall) + 2*(dy - dyBall)*(y - yBall) + 2*(dz - dzBall)*(z - zBall))^2)/(4*((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(5/2)) - ((z - zBall)*(2*(ddx - ddxBall)*(x - xBall) + 2*(ddy - ddyBall)*(y - yBall) + 2*(ddz - ddzBall)*(z - zBall) + 2*(dx - dxBall)^2 + 2*(dy - dyBall)^2 + 2*(dz - dzBall)^2))/(2*((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(3/2)))/(1 - (z - zBall)^2/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2))^(1/2) + (((dz - dzBall)/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(1/2) - ((z - zBall)*(2*(dx - dxBall)*(x - xBall) + 2*(dy - dyBall)*(y - yBall) + 2*(dz - dzBall)*(z - zBall)))/(2*((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^(3/2)))*((2*(dz - dzBall)*(z - zBall))/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2) - ((z - zBall)^2*(2*(dx - dxBall)*(x - xBall) + 2*(dy - dyBall)*(y - yBall) + 2*(dz - dzBall)*(z - zBall)))/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2)^2))/(2*(1 - (z - zBall)^2/((x - xBall)^2 + (y - yBall)^2 + (z - zBall)^2))^(3/2));...
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0];
%% System Dynamics %%
disp('Writing system Dynamics...');

%Symbolic Vectors
r = [x;y;z];
rBall = [xBall;yBall;zBall];
dr = [dx;dy;dz];
drBall = [dxBall;dyBall;dzBall];
ddr = [ddx;ddy;ddz];
ddrBall = [ddxBall;ddyBall;ddzBall];
eta = [phi;theta;psi];
etadot = [dphi;dtheta;dpsi];
etaddot = [ddphi;ddtheta;ddpsi];
taub = [tauphi; tautheta; taupsi];
eta_hat = [0      -eta(1) eta(2);
           eta(3)    0   -eta(1);
          -eta(2)  eta(1)      0];

rotx = [1          0         0;...
     0       cos(phi) -sin(phi);...
     0       sin(phi) cos(phi)];
 
roty = [cos(theta) 0 sin(theta);...
     0           1         0     ;...
     -sin(theta) 0 cos(theta)];
 
rotz = [cos(psi) -sin(psi)   0;...
      sin(psi)   cos(psi) 0;...
      0          0        1];
  
rotm = rotz*roty*rotx;
p = (rBall-r)/l;
e3 = [0;0;1];

% States
states = [r;dr;eta;etadot;rBall;drBall];
dstates = [dr;ddr;etadot;etaddot;drBall;ddrBall];

% Inputs
u = [f;taub];

eq1 = [dxBall;dyBall;dzBall] == drBall;
eq2 = (mQ+mL)*(ddrBall+g*e3) == (dot(p,f*rotm*e3)-mQ*l*(dot(dp,dp)))*p;
eq3 = dp == cross(w,p);
eq4 = mQ*l*dw == cross(-p,f*rotm*e3);
eq5 = [dphi;dtheta;dpsi] == etadot;
eq6 = J*etaddot+cross(eta,J*eta) == taub;
eqns = [eq1;eq2;eq3;eq4;eq5;eq6];
sol = solve(eqns,dstates);

dX = [sol.dx;
      sol.dy;
      sol.dz;
      sol.ddx;
      sol.ddy;
      sol.ddz;
      sol.dphi;   
      sol.dtheta;
      sol.dpsi;
      sol.ddphi;
      sol.ddtheta;
      sol.ddpsi;
      sol.dxBall;
      sol.dyBall;
      sol.dzBall;
      sol.ddxBall;
      sol.ddyBall;
      sol.ddzBall];

X = [x; y; z; dx; dy; dz; phi; theta; psi; dphi; dtheta; dpsi; xBall; yBall; zBall; dxBall; dyBall; dzBall];
U = [f;tauphi;tautheta;taupsi];

matlabFunction(dX ,'file','autoGen_dynamics.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'xBall', 'yBall', 'zBall','dxBall','dyBall','dzBall',...
     'T', 'tauphi', 'tautheta', 'taupsi'},...
     'outputs',{'dX'});
 
%% Linearization of Dynamics
disp('Writing linearization...');

A = simplify(jacobian(dX,X));
B = simplify(jacobian(dX,U));

matlabFunction(A,B,'file','autoGen_linearization.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall',...
     'T', 'tauphi', 'tautheta', 'taupsi'},...
     'outputs',{'A','B'});


% %% Derive Objective Function 
% disp('Writing objective function files...')
% 
% Z = [X;U];   % time-varying vector of inputs
% empty = sym('empty','real');   %Used for vectorization, user should pass a vector of zeros
% 
% %Minimize Torque
% syms R Q 'real'
% F = X'*Q*X + U'*R*U;
% [f, ~, fz, fzi, ~]  = computeGradients(F,Z,empty);
% 
% matlabFunction(f,fz,fzi,...
%     'file','autoGen_objective.m',...
%     'vars',{'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
%      'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall',...
%      'T', 'tauphi', 'tautheta', 'taupsi',...
%      'R','Q'});
% 
% %% Forward Kinematics
% disp('Writing forward kinematics files...')
% 
% rotx = [1    0             0;...
%         0 cos(phiBall) -sin(phi);...
%         0 sin(phi)  cos(phi)];
% roty = [cos(theta) 0 sin(theta);...
%         0          1          0;...
%        -sin(theta) 0 cos(theta)];
% rotz = [cos(psi) -sin(psi) 0;...
%         sin(psi)  cos(psi) 0;...
%         0         0        1];
% rotm = rotz*roty*rotx;
% 
% R1 = rotm*[0  param.rotor_offset 0]';
% R2 = rotm*[param.rotor_offset  0 0]';
% R3 = rotm*[0 -param.rotor_offset 0]';
% R4 = rotm*[-param.rotor_offset 0 0]';
% 
% P1 = [x,y,z]; %quadrotor COM
% P2 = P1 + [(cos(phiBall)*sin(thetaBall))/10 -sin(phiBall)/10  (cos(phiBall)*cos(thetaBall))/10 + 3/200]; %ball position
% P3 = P1 + [R1(1) R1(2) R1(3)]; %rotor 1 position
% P4 = P1 + [R2(1) R2(2) R2(3)]; %rotor 2 position
% P5 = P1 + [R3(1) R3(2) R3(3)]; %rotor 3 position
% P6 = P1 + [R4(1) R4(2) R4(3)]; %rotor 4 position
% 
% P = [P1; P2; P3; P4; P5; P6];
% 
% % Used for plotting and animation
% matlabFunction(P,'file','autoGen_forwardKinematics.m','vars',...
%     {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
%      'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall'},...
%      'outputs',{'P'});
% 
% %% Helper Functions
% function [m, mi, mz, mzi, dim] = computeGradients(M,z,empty)
%     % This function computes the gradients of a matrix M with respect the the
%     % variables in z, and then returns both the matrix and its gradient as
%     % column vectors of their non-zero elements, along with the linear indicies
%     % to unpack them. It also simplifies m and mz.
%     %
%     % INPUTS:
%     %   M = [na, nb] = symbolic matrix
%     %   z = [nc, 1] = symbolic vector
%     %
%     % OUTPUTS:
%     %   m = [nd, 1] = symbolic vector of non-zero elements in M
%     %   i = [nd, 1] = linear indicies to map m --> [na,nb] matrix
%     %   mz = [ne, 1] = symbolic vector of non-zero elements in Mz
%     %   iz = [ne, 1] = linear indicies to map mz --> [na,nb,nc] array
%     %   dim = [3,1] = [na,nb,nc] = dimensions of 3d version of mz
%     %
% 
%     [na, nb] = size(M);
%     nc = size(z,1);
%     M = simplify(M);
% 
%     mz2 = jacobian(M(:),z);  %Compute jacobian of M, by first reshaping M to be a column vector
%     mz3 = reshape(mz2,na,nb,nc); %Expand back out to a three-dimensional array
%     mz3 = simplify(mz3);
% 
%     % Extract non-zero elements to a column vector:
%     mi = find(M);
%     m = M(mi);
%     mzi = find(mz3);
%     mz = mz3(mzi); mz = mz(:);  %Collapse to a column vector
%     dim = [na,nb,nc];
% 
%     % Pad any constant terms with "empty" to permit vectorization:
%     m = vectorizeHack(m, z, empty);
%     mz = vectorizeHack(mz, z, empty);
% end
% 
% function x = vectorizeHack(x, z, empty)
%     % This function searches for any elements of x that are not dependent on
%     % any element of z. In this case, the automatically generated code will
%     % fail to vectorize properly. One solution is to add an array of zeros
%     % (empty) to the element.
%     %
%     % x = column vector of symbolic expressions
%     % z = column vector of symbolic variables
%     % z = symbolic variable, which the user will set equal to zero.
%     %
% 
%     % Compute dependencies
%     g = jacobian(x,z);
% 
%     % Check for rows of x with no dependence on z
%     [n,m] = size(g);
%     idxConst = true(n,1);
%     for i=1:n
%         for j=1:m
%             if ~isequal(sym(0),g(i,j))
%                 idxConst(i) = false;
%                 break;
%             end
%         end
%     end
% 
%     % Add empty to those enteries
%     x(idxConst) = x(idxConst) + empty;
% end