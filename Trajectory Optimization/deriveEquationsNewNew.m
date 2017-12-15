clear; close all; clc; 
%% Derive Quadrotor with Inverted Pendulum Dynamics
syms x y z dx dy dz ddx ddy ddz phi theta psi dphi dtheta dpsi ddphi ddtheta ddpsi T tauphi tautheta taupsi xBall yBall zBall dxBall dyBall dzBall ddxBall ddyBall ddzBall 'real'
syms X(t) Y(T) Z(T) XBall(t) YBall(t) ZBall(t) tension
dXBall = diff(XBall,t);
ddXBall = diff(dXBall,t);
dYBall = diff(YBall,t);
ddYBall = diff(dYBall,t);
dZBall = diff(ZBall,t);
ddZBall = diff(dZBall,t);
dX(t) = diff(X(t),t);
ddX(t) = diff(dX(t),t);
dY(t) = diff(Y(t),t);
ddY(t) = diff(dY(t),t);
dZ(t) = diff(Z(t),t);
ddZ(t) = diff(dZ(t),t);

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

%Symbolic Vectors
r = [x;y;z];
R = [X(t);Y(t);Z(t)];
rBall = [xBall;yBall;zBall];
dr = [dx;dy;dz];
drBall = [dxBall;dyBall;dzBall];
ddr = [ddx;ddy;ddz];
ddrBall = [ddxBall;ddyBall;ddzBall];
eta = [phi;theta;psi];
etadot = [dphi;dtheta;dpsi];
etaddot = [ddphi;ddtheta;ddpsi];
taub = [tauphi; tautheta; taupsi];

Rx = [1          0         0;...
     0       cos(phi) -sin(phi);...
     0       sin(phi) cos(phi)];
 
Ry = [cos(theta) 0 sin(theta);...
     0           1         0     ;...
     -sin(theta) 0 cos(theta)];
 
Rz = [cos(psi) -sin(psi)   0;...
      sin(psi)   cos(psi) 0;...
      0          0        1];
rot = Rz * Ry * Rx;
p = (rBall-r)/l;
G = [0; 0; g];

constraint = (XBall(t)-X(t))^2 + (YBall(t)-Y(t))^2 + (ZBall(t)-Z(t))^2;
dconstraint = diff(constraint);
ddconstraint = diff(dconstraint);

constraint = subs(constraint,{XBall,dXBall,ddXBall,YBall,dYBall,ddYBall,ZBall,dZBall,ddZBall,X,dX,ddX,Y,dY,ddY,Z,dZ,ddZ},{xBall,dxBall,ddxBall,yBall,dyBall,ddyBall,zBall,dzBall,ddzBall,x,dx,ddx,y,dy,ddy,z,dz,ddz});
dconstraint = subs(dconstraint,{XBall,dXBall,ddXBall,YBall,dYBall,ddYBall,ZBall,dZBall,ddZBall,X,dX,ddX,Y,dY,ddY,Z,dZ,ddZ},{xBall,dxBall,ddxBall,yBall,dyBall,ddyBall,zBall,dzBall,ddzBall,x,dx,ddx,y,dy,ddy,z,dz,ddz});
ddconstraint = subs(ddconstraint,{XBall,dXBall,ddXBall,YBall,dYBall,ddYBall,ZBall,dZBall,ddZBall,X,dX,ddX,Y,dY,ddY,Z,dZ,ddZ},{xBall,dxBall,ddxBall,yBall,dyBall,ddyBall,zBall,dzBall,ddzBall,x,dx,ddx,y,dy,ddy,z,dz,ddz});

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
states = [r;dr;eta;etadot;rBall;drBall];
dstates = [dr;ddr;etadot;etaddot;drBall;ddrBall];

% Inputs
u = [T;taub];

eq1 = [dx;dy;dz] == dr;
eq2 = ddr == rot*[0;0;T/m] - G + ((tension*p)/m);
eq3 = [dphi;dtheta;dpsi] == etadot;
eq4 = etaddot == J\(taub - C*etadot);
eq5 = ddrBall == -((tension*p)/mL) - G;
eq6 = ddconstraint == 0;
eqs = [eq1;eq2;eq3;eq4;eq5;eq6];

sol = solve(eqs,dstates);

dX = [sol.dr;
      sol.ddr;
      sol.etadot;
      sol.etaddot;
      sol.drBall;
      sol.ddrBall];

X = [x; y; z; dx; dy; dz; phi; theta; psi; dphi; dtheta; dpsi; xBall; yBall; zBall; dxBall; dyBall; dzBall];
U = [T;tauphi;tautheta;taupsi];

matlabFunction(dX ,'file','autoGen_dynamics.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'xBall', 'yBall', 'zBall','dxBall','dyBall','dzBall',...
     'T', 'tauphi', 'tautheta', 'taupsi'},...
     'outputs',{'dX'});
 
%% Linearization of Dynamics
A = simplify(jacobian(dX,X));
B = simplify(jacobian(dX,U));

matlabFunction(A,B,'file','autoGen_linearization.m','vars',...
    {'x', 'y', 'z', 'dx', 'dy', 'dz', 'phi', 'theta', 'psi', 'dphi', 'dtheta', 'dpsi', ...
     'phiBall', 'xBall', 'yBall', 'zBall','dxBall','dyBall','dzBall',...
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
     'phiBall', 'xBall', 'yBall', 'zBall','dxBall','dyBall','dzBall',...
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