function P = getPoints(X)
% This function computes the node positions of the 
% quadrotor and swinging load
%
% INPUTS:
%   q = [16, n] = states
%   p = parameters struct
%
% OUTPUTS:
%   P = [6, n] = node positions [x;y;x;y;...]

P = autoGen_forwardKinematics(...
    X(1,:),X(2,:),X(3,:),X(4,:),...
    X(5,:),X(6,:),X(7,:),X(8,:),...
    X(9,:),X(10,:),X(11,:),X(12,:),...
    X(13,:),X(14,:),X(15,:),X(16,:));
end