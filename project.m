tstep = 300; %Time to simulate for
%Linearized Longitudinal Dynamics
num_long = [2.423,.1097];
den_long = [1,.3537,.1394,.002024];
long_dynamics = tf(num_long,den_long);
figure(1)
step(long_dynamics,tstep)
S1 = stepinfo(long_dynamics);
title('Open Loop Longitudinal Step Response')

%Linearized Rotational dynamics
num_rot = [.00835,.001745];
den_rot = [1,4.641*(10^-5),.0002839];
rot_dynamics = tf(num_rot,den_rot);
figure(2)
step(rot_dynamics,tstep*1000)
S2 = stepinfo(rot_dynamics);
title('Open Loop Rotational Step Response')

%Forward mode dynamics
num_fwd = [.0002557,-.02031,.05613];
den_fwd = [1,.4456,27.55,4.316];
fwd_dynamics = tf(num_fwd,den_fwd);
figure(3)
step(fwd_dynamics,tstep/3)
S3 = stepinfo(fwd_dynamics)
title('Open Loop Forward Step Response')

figure(4)
rlocus(fwd_dynamics)


