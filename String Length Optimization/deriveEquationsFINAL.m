%% Derive Quadrotor with Inverted Pendulum Dynamics %%
clc; clear; close all;

%% Symbolics
syms xQ yQ zQ dxQ dyQ dzQ ddxQ ddyQ ddzQ phi theta psi dphi dtheta dpsi ddphi ddtheta ddpsi f tauphi tautheta taupsi 'real'
syms xL yL zL dxL dyL dzL ddxL ddyL ddzL 'real'
syms T 'real'

%% Parameters
param = getParameters();
Ixx = param.Ixx;
Iyy = param.Iyy;
Izz = param.Izz;
mQ = param.m; %quad mass
g = param.g; %gravity
mL = param.mL; %load mass
l = param.l; %string length

%% States %%
X = [xQ yQ zQ dxQ dyQ dzQ phi theta psi dphi dtheta dpsi xL yL zL dxL dyL dzL]'; %states
U = [f tauphi tautheta taupsi]'; %inputs

%% Knowns
XQ = [xQ yQ zQ]'; %quad position
XL = [xL yL zL]'; %load position
etaQ = [phi theta psi]'; %quad orientation

dXL = [dxL dyL dzL]'; %load velocity

detaQ = [dphi dtheta dpsi]'; %quad angular rates

p = (XQ-XL)/l; %unit vector from quad to load
e3 = [0 0 1]'; %cartesian z unit vector
M = [tauphi tautheta taupsi]'; %moment vector on quad
J = diag([Ixx Iyy Izz]); %quad inertia tensor

rotx = [1           0         0;...
        0        cos(phi) -sin(phi);...
        0        sin(phi) cos(phi)];
roty = [cos(theta)  0     sin(theta);...
        0           1         0    ;...
        -sin(theta) 0   cos(theta)];
rotz = [cos(psi) -sin(psi)   0;...
        sin(psi)  cos(psi)   0;...
        0          0         1]; 
R = rotz*roty*rotx; %rotation matrix from inertial to quad body frame

%% Unknowns
ddetaQ = [ddphi ddtheta ddpsi]'; %quad rotational accelerations
ddXQ = [ddxQ ddyQ ddzQ]'; %quad linear accelerations
ddXL = [ddxL ddyL ddzL]'; %load linear accelerations

% %% Solve for Tension
% syms T 'real'
% eqt = T*p(3) == ((mL/l)*dxL.^2) + mL*g;
% sol_T = solve(eqt,T);
% T = -sol_T;
% % T = 0; 

%% Solve for Linear Acceleration
eq1a = mQ*ddXQ == f*R*e3 - mQ*g*e3 + T*p;
sol_ddXQ = solve(eq1a,ddXQ);

%% Solve for Angular Acceleration
eq2a = J*ddetaQ + cross(detaQ,J*detaQ) == M;
sol_ddetaQ = solve(eq2a,ddetaQ);

%% Solve for Load Acceleration
eq3a = mL*ddXL == -T*p - mL*g*e3;
sol_ddXL = solve(eq3a,ddXL);

dX = [dxQ;
      dyQ;
      dzQ;
      sol_ddXQ.ddxQ;
      sol_ddXQ.ddyQ;
      sol_ddXQ.ddzQ;
      dphi;   
      dtheta;
      dpsi;
      sol_ddetaQ.ddphi;
      sol_ddetaQ.ddtheta;
      sol_ddetaQ.ddpsi;
      dxL;
      dyL;
      dzL;
      sol_ddXL.ddxL;
      sol_ddXL.ddyL;
      sol_ddXL.ddzL];
 
% Write to File
matlabFunction(dX ,'file','autoGen_dynamics_with_tension.m','vars',...
    {'xQ', 'yQ', 'zQ', 'dxQ','dyQ','dzQ','phi','theta','psi','dphi','dtheta','dpsi',...
     'xL', 'yL', 'zL', 'dxL','dyL','dzL',...
     'f', 'tauphi', 'tautheta', 'taupsi','T',},...
     'outputs',{'dX'});
 
%% Linearization of Dynamics
A = simplify(jacobian(dX(1:12),X(1:12)));
B = simplify(jacobian(dX(1:12),U));

matlabFunction(A,B,'file','autoGen_linearization_with_tension.m','vars',...
    {'xQ', 'yQ', 'zQ', 'dxQ','dyQ','dzQ','phi','theta','psi','dphi','dtheta','dpsi',...
     'f', 'tauphi', 'tautheta', 'taupsi','T',},...
     'outputs',{'A','B'});

%% Derive Objective Function 
Z = [X;U];   % time-varying vector of inputs
empty = sym('empty','real');   %Used for vectorization, user should pass a vector of zeros

%Minimize Torque
syms R Q 'real'
F = X'*Q*X + U'*R*U;
[f, ~, fz, fzi, ~]  = computeGradients(F,Z,empty);

matlabFunction(f,fz,fzi,...
    'file','autoGen_objective_with_tension.m',...
    'vars',{'xQ', 'yQ', 'zQ', 'dxQ','dyQ','dzQ','phi','theta','psi','dphi','dtheta','dpsi',...
     'xL', 'yL', 'zL', 'dxL','dyL','dzL',...
     'f', 'tauphi', 'tautheta', 'taupsi','R','Q','T'});

%% Forward Kinematics
%Quad Rotation Matrix
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

%Rotor Positions
R1 = rotm*[0  param.rotor_offset 0]';
R2 = rotm*[param.rotor_offset  0 0]';
R3 = rotm*[0 -param.rotor_offset 0]';
R4 = rotm*[-param.rotor_offset 0 0]';

P1 = [xQ,yQ,zQ]; %quadrotor COM
P2 = [xL,yL,zL]; %ball position
P3 = P1 + [R1(1) R1(2) R1(3)]; %rotor 1 position
P4 = P1 + [R2(1) R2(2) R2(3)]; %rotor 2 position
P5 = P1 + [R3(1) R3(2) R3(3)]; %rotor 3 position
P6 = P1 + [R4(1) R4(2) R4(3)]; %rotor 4 position

P = [P1; P2; P3; P4; P5; P6];

% Used for plotting and animation
matlabFunction(P,'file','autoGen_forwardKinematics_with_tension.m','vars',...
    {'xQ', 'yQ', 'zQ', 'dxQ','dyQ','dzQ','phi','theta','psi','dphi','dtheta','dpsi',...
     'xL', 'yL', 'zL', 'dxL','dyL','dzL','l'},...
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