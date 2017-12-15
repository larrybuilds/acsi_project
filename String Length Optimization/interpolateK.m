%Interpolate State and Input at time t
function k = interpolateK(t,p)
    K = p.K;
    k = zeros(4,16);
    for i=1:4
        for j=1:16
            k(i,j) = interp1(p.Tspan,K(:,i,j),t);
        end
    end
end