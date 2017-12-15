function dP = ricatti(t,Pvec,Q,R,x_nominal,u_nominal,p)    
    %Unpack
    P = reshape(Pvec,length(Q),length(Q));
    
    %Interpolate Trajectory
    [x_nom,u_nom] = interpolateTrajectory(x_nominal,u_nominal,t,p);
    
    %Get A(t),B(t)
    [A,B] = linearize(x_nom,u_nom);
    
    %Solve Differential Equation
    dP = -(Q - P*B*inv(R)*B'*P + P*A + A'*P);
    dP = reshape(dP,length(Q)*length(Q),1);
end
