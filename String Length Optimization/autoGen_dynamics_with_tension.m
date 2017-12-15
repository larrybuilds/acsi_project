function dX = autoGen_dynamics_with_tension(xQ,yQ,zQ,dxQ,dyQ,dzQ,phi,theta,psi,dphi,dtheta,dpsi,xL,yL,zL,dxL,dyL,dzL,f,tauphi,tautheta,taupsi,T)
%AUTOGEN_DYNAMICS_WITH_TENSION
%    DX = AUTOGEN_DYNAMICS_WITH_TENSION(XQ,YQ,ZQ,DXQ,DYQ,DZQ,PHI,THETA,PSI,DPHI,DTHETA,DPSI,XL,YL,ZL,DXL,DYL,DZL,F,TAUPHI,TAUTHETA,TAUPSI,T)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    13-Dec-2017 20:47:26

t2 = cos(psi);
t3 = sin(phi);
t4 = cos(phi);
t5 = sin(psi);
t6 = sin(theta);
t7 = xL.*1.0e1;
t8 = t7-xQ.*1.0e1;
t9 = yL.*1.0e1;
t10 = t9-yQ.*1.0e1;
t11 = zL.*1.0e1;
t12 = t11-zQ.*1.0e1;
dX = [dxQ;dyQ;dzQ;T.*t8.*(-1.0e3./3.7e1)+f.*(t3.*t5+t2.*t4.*t6).*(1.0e3./3.7e1);T.*t10.*(-1.0e3./3.7e1)-f.*(t2.*t3-t4.*t5.*t6).*(1.0e3./3.7e1);T.*t12.*(-1.0e3./3.7e1)+f.*t4.*cos(theta).*(1.0e3./3.7e1)-9.81e2./1.0e2;dphi;dtheta;dpsi;tauphi.*4.175191014988935e4-dpsi.*dtheta.*3.50549037618471e-1;tautheta.*4.175191014988935e4+dphi.*dpsi.*3.50549037618471e-1;taupsi.*3.091476798466627e4;dxL;dyL;dzL;T.*t8.*1.0e3;T.*t10.*1.0e3;T.*t12.*1.0e3-9.81e2./1.0e2];