function dx = dynamicsSimLQR(t,x,xd,u_nom,p)
    % Computes the first-order dynamics of the quadrotor with suspended
    % load, with TV-LQR feedback controller.
    %
    % INPUTS:
    %   x = [16,n] = first-order state = [x; dx; R; dR; l; dL];
    %   u = [4, n] = input force/torque vector
    
    %Find Error
    xbar = xd - x;
    
    %Feedback Input
    u = p.K*xbar + u_nom;
    
    dx = autoGen_dynamics(...
        x(1,:),x(2,:),x(3,:),x(4,:),...
        x(5,:),x(6,:),x(7,:),x(8,:),...
        x(9,:),x(10,:),x(11,:),x(12,:),...
        x(13,:),x(14,:),x(15,:),x(16,:),...
        u(1,:),u(2,:),u(3,:),u(4,:));
end