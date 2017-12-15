clear; close all; clc; 
%% Derive Quadrotor with Inverted Pendulum Dynamics
syms x y z dx dy dz ddx ddy ddz phi theta psi dphi dtheta dpsi ddphi ddtheta ddpsi T tauphi tautheta taupsi 'real'

%% Parameters
param = getParameters();
Ixx = param.Ixx;
Iyy = param.Iyy;
Izz = param.Izz;
m = param.m;
g = param.g;
mL = param.mL;
z_offset = param.z_offset;
l = param.l;

%% Quadcopter %%
disp('Writing quadrotor dynamics files...')

% Gravity acceleration
Ga = -g * [0; 0; 1];

% Load Torques
Ry = [cos(theta) 0 sin(theta);...
     0           1         0     ;...
     -sin(theta) 0 cos(theta)];
Rx = [1          0         0;...
     0       cos(phi) -sin(phi);...
     0       sin(phi) cos(phi)];
F_H = [0;0;mL*g];
pH = Ry*Rx*[0;0;z_offset];
T_H = cross(pH,F_H);

%Symbolic Vectors
r = [x;y;z];
rdot = [dx;dy;dz];
rddot = [ddx;ddy;ddz];
eta = [phi;theta;psi];
etadot = [dphi;dtheta;dpsi];
etaddot = [ddphi;ddtheta;ddpsi];
taub = [tauphi; tautheta; taupsi];

% Thrust acceleration
angM = [cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
        sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
        cos(theta)*cos(phi)];

Ta = T/m * angM;

Sphi = sin(phi); Stheta = sin(theta); Spsi = sin(psi);
Cphi = cos(phi); Ctheta = cos(theta); Cpsi = cos(psi);

% Coriolis Matrix
C11 = 0;
C12 = (Iyy - Izz) * (dtheta*Cphi*Sphi + dpsi*Sphi^2*Ctheta) + (Izz - Iyy)*dpsi*Cphi^2*Ctheta - Ixx*dpsi*Ctheta;
C13 = (Izz - Iyy) * dpsi*Cphi*Sphi*Ctheta^2;
C21 = (Izz - Iyy) * (dtheta*Cphi*Sphi + dpsi*Sphi*Ctheta) + (Iyy - Izz)*dpsi*Cphi^2*Ctheta - Ixx*dpsi*Ctheta;
C22 = (Izz - Iyy) * dphi*Cphi*Sphi;
C23 = -Ixx * dpsi * Stheta * Ctheta + Iyy * dpsi * Sphi^2 * Stheta * Ctheta + Izz * dpsi * Cphi^2 * Stheta * Ctheta;
C31 = (Iyy - Izz) * dpsi * Ctheta^2 * Sphi * Cphi - Ixx * dtheta * Ctheta;
C32 = (Izz - Iyy) * (dtheta * Cphi * Sphi * Stheta + dphi * Sphi^2 * Ctheta) + (Iyy - Izz) * dphi * Cphi^2 * Ctheta...
        + Ixx * dpsi * Stheta * Ctheta - Iyy * dpsi * Sphi^2 * Stheta * Ctheta - Izz * psi * Cphi^2 * Stheta * Ctheta;
C33 = (Iyy - Izz) * dphi * Cphi * Sphi * Ctheta^2 - Iyy * dtheta * Sphi^2 * Ctheta * Stheta ...
        - Izz * dtheta * Cphi^2 * Ctheta * Stheta + Ixx * dtheta * Ctheta * Stheta;

C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];    

% Inertia Matrix
J11 = Ixx; J12 = 0; J13 = -Ixx*Stheta;
J21 = 0;
J22 = Iyy * Cphi^2 + Izz * Sphi^2;
J23 = (Iyy - Izz) * Cphi * Sphi * Ctheta;
J31 = -Ixx * Stheta;
J32 = (Iyy - Izz)*Cphi*Sphi*Ctheta;
J33 = Ixx * Stheta^2 + Iyy * Sphi^2 * Ctheta^2 + Izz * Cphi^2*Ctheta^2;
J = [J11 J12 J13; J21 J22 J23; J31 J32 J33];  

% States
states = [r;rdot;eta;etadot];

% Inputs
u = [T;taub];

dXquad = [rdot;
          T/(m+mL) * angM - [0;0;g];
          etadot;
          J\(taub - C*etadot)];
     
ddx = dXquad(4);
ddy = dXquad(5);
ddz = dXquad(6);
ddphi = dXquad(10);
ddtheta = dXquad(11);
ddpsi = dXquad(12);

%% Pendulum %%
disp('Writing pendulum dynamics files...')

%Time Variables
syms PhiBall(t) ThetaBall(t)
dPhiBall(t) = diff(PhiBall,t);
ddPhiBall(t) = diff(dPhiBall,t);
dThetaBall(t) = diff(ThetaBall,t);
ddThetaBall(t) = diff(dThetaBall,t);

%Load Position
Ry_L = [cos(ThetaBall) 0 sin(ThetaBall);...
     0               1         0     ;...
     -sin(ThetaBall) 0 cos(ThetaBall)];
Rx_L = [1              0         0;...
     0       cos(PhiBall) -sin(PhiBall);...
     0       sin(PhiBall) cos(PhiBall)];
pL = Ry_L * Rx_L * [0;0;l];
pH = [0;0;z_offset];
p = pL+pH;
dp = diff(p,t);
ddp = diff(dp,t);

%Substitute Symbolics
syms thetaBall dthetaBall ddthetaBall phiBall dphiBall ddphiBall 'real'
p = subs(p,{PhiBall,dPhiBall,ddPhiBall,ThetaBall,dThetaBall,ddThetaBall},{phiBall,dphiBall,ddphiBall,thetaBall,dthetaBall,ddthetaBall});
dp = subs(dp,{PhiBall,dPhiBall,ddPhiBall,ThetaBall,dThetaBall,ddThetaBall},{phiBall,dphiBall,ddphiBall,thetaBall,dthetaBall,ddthetaBall});
ddp = subs(ddp,{PhiBall,dPhiBall,ddPhiBall,ThetaBall,dThetaBall,ddThetaBall},{phiBall,dphiBall,ddphiBall,thetaBall,dthetaBall,ddthetaBall});
pL = subs(pL,{PhiBall,dPhiBall,ddPhiBall,ThetaBall,dThetaBall,ddThetaBall},{phiBall,dphiBall,ddphiBall,thetaBall,dthetaBall,ddthetaBall});
pH = subs(pH,{PhiBall,dPhiBall,ddPhiBall,ThetaBall,dThetaBall,ddThetaBall},{phiBall,dphiBall,ddphiBall,thetaBall,dthetaBall,ddthetaBall});

v1 = [dx;dy;dz];
dv1 = [ddx;ddy;ddz];
v2 = [dphi;dtheta;dpsi];
dv2 = [ddphi;ddtheta;ddpsi];

vL = v1 + dp + cross(v2,p);
dvL = dv1 + ddp + cross(dv2,p) + cross(2*v2,dp) + cross(v2,cross(v2,p));

fGL = [0;0;-mL*g]; %force of gravity

%torque equilibrium around suspension point
ddq = [ddphiBall;ddthetaBall];
EOM = cross(-pL,(-mL*dvL + fGL)) == 0;
sol = solve(EOM,ddq);
ddphiBallSolution = simplify(sol.ddphiBall);
ddthetaBallSolution = simplify(sol.ddthetaBall);

dXpend = [dphiBall;
         dthetaBall;
         ddphiBallSolution;
         ddthetaBallSolution];

%% System Dynamics
dX = [dXquad;...
      dXpend];

X = [x; y; z; dx; dy; dz; phi; theta; psi; dphi; dtheta; dpsi; phiBall; thetaBall; dphiBall; dthetaBall];
U = [T;tauphi;tautheta;taupsi];

matlabFunction(dX ,'file','autoGen_dynamics.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall',...
     'T', 'tauphi', 'tautheta', 'taupsi'},...
     'outputs',{'dX'});
 
%% Linearization of Dynamics
A = simplify(jacobian(dX,X));
B = simplify(jacobian(dX,U));

matlabFunction(A,B,'file','autoGen_linearization.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall',...
     'T', 'tauphi', 'tautheta', 'taupsi'},...
     'outputs',{'A','B'});


%% Derive Objective Function 
disp('Writing objective function files...')

Z = [X;U];   % time-varying vector of inputs
empty = sym('empty','real');   %Used for vectorization, user should pass a vector of zeros

%Minimize Torque
syms R Q 'real'
F = X'*Q*X + U'*R*U;
[f, ~, fz, fzi, ~]  = computeGradients(F,Z,empty);

matlabFunction(f,fz,fzi,...
    'file','autoGen_objective.m',...
    'vars',{'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall',...
     'T', 'tauphi', 'tautheta', 'taupsi','R','Q'});

%% Forward Kinematics
disp('Writing forward kinematics files...')

rotx = [1    0             0;...
        0 cos(phi) -sin(phi);...
        0 sin(phi)  cos(phi)];
roty = [cos(theta) 0 sin(theta);...
        0          1          0;...
       -sin(theta) 0 cos(theta)];
rotz = [cos(psi) -sin(psi) 0;...
        sin(psi)  cos(psi) 0;...
        0         0        1];
rotm = rotz*roty*rotx;

R1 = rotm*[0  param.rotor_offset 0]';
R2 = rotm*[param.rotor_offset  0 0]';
R3 = rotm*[0 -param.rotor_offset 0]';
R4 = rotm*[-param.rotor_offset 0 0]';

P1 = [x,y,z]; %quadrotor COM
P2 = P1 + [(cos(phiBall)*sin(thetaBall))/10 -sin(phiBall)/10  (cos(phiBall)*cos(thetaBall))/10 + 3/200]; %ball position
P3 = P1 + [R1(1) R1(2) R1(3)]; %rotor 1 position
P4 = P1 + [R2(1) R2(2) R2(3)]; %rotor 2 position
P5 = P1 + [R3(1) R3(2) R3(3)]; %rotor 3 position
P6 = P1 + [R4(1) R4(2) R4(3)]; %rotor 4 position

P = [P1; P2; P3; P4; P5; P6];

% Used for plotting and animation
matlabFunction(P,'file','autoGen_forwardKinematics.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'thetaBall', 'dphiBall', 'dthetaBall'},...
     'outputs',{'P'});

%% Helper Functions
function [m, mi, mz, mzi, dim] = computeGradients(M,z,empty)
    % This function computes the gradients of a matrix M with respect the the
    % variables in z, and then returns both the matrix and its gradient as
    % column vectors of their non-zero elements, along with the linear indicies
    % to unpack them. It also simplifies m and mz.
    %
    % INPUTS:
    %   M = [na, nb] = symbolic matrix
    %   z = [nc, 1] = symbolic vector
    %
    % OUTPUTS:
    %   m = [nd, 1] = symbolic vector of non-zero elements in M
    %   i = [nd, 1] = linear indicies to map m --> [na,nb] matrix
    %   mz = [ne, 1] = symbolic vector of non-zero elements in Mz
    %   iz = [ne, 1] = linear indicies to map mz --> [na,nb,nc] array
    %   dim = [3,1] = [na,nb,nc] = dimensions of 3d version of mz
    %

    [na, nb] = size(M);
    nc = size(z,1);
    M = simplify(M);

    mz2 = jacobian(M(:),z);  %Compute jacobian of M, by first reshaping M to be a column vector
    mz3 = reshape(mz2,na,nb,nc); %Expand back out to a three-dimensional array
    mz3 = simplify(mz3);

    % Extract non-zero elements to a column vector:
    mi = find(M);
    m = M(mi);
    mzi = find(mz3);
    mz = mz3(mzi); mz = mz(:);  %Collapse to a column vector
    dim = [na,nb,nc];

    % Pad any constant terms with "empty" to permit vectorization:
    m = vectorizeHack(m, z, empty);
    mz = vectorizeHack(mz, z, empty);
end

function x = vectorizeHack(x, z, empty)
    % This function searches for any elements of x that are not dependent on
    % any element of z. In this case, the automatically generated code will
    % fail to vectorize properly. One solution is to add an array of zeros
    % (empty) to the element.
    %
    % x = column vector of symbolic expressions
    % z = column vector of symbolic variables
    % z = symbolic variable, which the user will set equal to zero.
    %

    % Compute dependencies
    g = jacobian(x,z);

    % Check for rows of x with no dependence on z
    [n,m] = size(g);
    idxConst = true(n,1);
    for i=1:n
        for j=1:m
            if ~isequal(sym(0),g(i,j))
                idxConst(i) = false;
                break;
            end
        end
    end

    % Add empty to those enteries
    x(idxConst) = x(idxConst) + empty;
end