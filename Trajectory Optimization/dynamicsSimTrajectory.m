function dx = dynamicsSimTrajectory(t,x,x_nom,u_nom,p)
    % Computes the first-order dynamics of the quadrotor with suspended
    % load, with TV-LQR feedback controller.
    %
    % INPUTS:
    %   x = [16,n] = first-order state = [x; dx; R; dR; l; dL];
    %   u = [4, n] = input force/torque vector
    
    %Interpolate x_nom,u_nom
    [x0,u0] = interpolateTrajectory(x_nom,u_nom,t,p);
    
    %Interpolate K
    K = interpolateK(t,p);
    
    %Find Error
    xbar = x - x0;
    
%     u0 = [p.g*(p.m + p.mL);0;0;0];

    %Feedback Input
    u_star = u0 - K*xbar;
    
    dx = autoGen_dynamics(...
        x(1,:),x(2,:),x(3,:),x(4,:),...
        x(5,:),x(6,:),x(7,:),x(8,:),...
        x(9,:),x(10,:),x(11,:),x(12,:),...
        x(13,:),x(14,:),x(15,:),x(16,:),...
        u_star(1,:),u_star(2,:),u_star(3,:),u_star(4,:));
end