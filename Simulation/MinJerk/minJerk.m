function traj = minJerk(x0,xf,t,numPoints)
    T = t;
    
    t = linspace(0,t,numPoints);
    
    for i=1:3
        %Find Coefficients
        a0 = x0(i);
        a1 = 0;
        a2 = 0;
        a3 =-(20*x0(i) - 20*xf(i))/(2*T^3);
        a4 =(30*x0(i) - 30*xf(i))/(2*T^4);
        a5 =-(12*x0(i) - 12*xf(i))/(2*T^5);

        %Build Trajectory
        x(:,i) = a5*t.^5 + a4*t.^4 + a3*t.^3 + a2*t.^2 + a1*t + a0;
    end
    
    traj = x;
end