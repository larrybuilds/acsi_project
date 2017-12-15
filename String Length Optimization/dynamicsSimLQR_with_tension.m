function dx = dynamicsSimLQR_with_tension(t,x,xd,u_nom,p)
    % Computes the first-order dynamics of the quadrotor with suspended
    % load, with TV-LQR feedback controller.
    %
    % INPUTS:
    %   x = [18,n] = first-order state = [x; dx; R; dR; l; dL];
    %   u = [4, n] = input force/torque vector
    global dXL
    
    dXL = [dXL; [x(16) x(17) x(18) t]];

    % Finite Difference for ball Acceleration
    t
    [a,~] = size(dXL);
    if a>=2
        if t>dXL(end-1,4)
            dt = dXL(end,4) - dXL(end-1,4);
            ddXL = (dXL(end,1:3)-dXL(end-1,1:3)) / dt;
            ddXL = ddXL';
        else
            ddXL = [0 0 0]';
        end
    else
        ddXL = [0 0 0]';
    end
    
    % String Tension
    e3 = [0 0 1]';
    T = -norm(p.mL*(ddXL + p.g*e3),2);
    
    %Find Error
    xbar = xd - x(1:12);
    
    %Feedback Input
    u = p.K*xbar + u_nom;
%     u = u_nom;
    
    dx = autoGen_dynamics_with_tension(...
        x(1,:),x(2,:),x(3,:),x(4,:),...
        x(5,:),x(6,:),x(7,:),x(8,:),...
        x(9,:),x(10,:),x(11,:),x(12,:),...
        x(13,:),x(14,:),x(15,:),x(16,:),x(17,:),x(18,:),...
        u(1,:),u(2,:),u(3,:),u(4,:),T);
end