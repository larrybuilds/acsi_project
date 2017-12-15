syms xf x0 'real'


thetad0 = 0;
thetadd0 = 0;
thetadf = 0;
thetaddf = 0;

a0 = x0;
a1 = thetad0;
a2 = 0.5 * thetadd0;
a3 =(1/(2*T^3)) * (20 * (xf - x0) - (8 * thetadf+ 12*thetad0 )*T - (3 * thetaddf - thetadd0 )*T^2 );
a4 =(1/(2*T^4)) * (30 * (x0 - xf) + (14 * thetadf+ 16*thetad0 )*T + (3 * thetaddf - 2*thetadd0 )*T^2 );
a5 =(1/(2*T^5)) * (12 * (xf - x0) - 6*(thetadf+ thetad0 )*T - (thetaddf - thetadd0 )*T^2 );
