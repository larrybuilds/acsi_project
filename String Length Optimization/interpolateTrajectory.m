%Interpolate State and Input at time t
function [x,u] = interpolateTrajectory(X,U,t,p)
    x = zeros(16,1);
    for i=1:16
        x(i) = interp1(p.Tspan,X(i,:),t);
    end
    u = zeros(4,1);
    for i=1:4
        u(i) = interp1(p.Tspan,U(i,:),t);
    end
end