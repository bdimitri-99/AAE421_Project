clear
clc
close all
tstep = 300;

%Linearized Longitudinal Dynamics
num_long = [2.423,.1097];
den_long = [1,.3537,.1394,.002024];
long_dynamics = tf(num_long,den_long);%Longitudinal dynamics transfer function
figure(1)
subplot(2,1,1)
[y,t] = step(long_dynamics,tstep); %Open loop step response
plot(t,y)
S1 = stepinfo(long_dynamics); %Open loop step response data
ess_long = y(end)-1;
title('Open Loop Longitudinal Step Response')
fprintf('The longitudinal open loop system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S1.Overshoot,S1.SettlingTime,ess_long)

subplot(2,1,2)
Kp_long = 38;
Ki_long = 16; %Gains for longitudinal dynamics closed loop step response
Kd_long = 74;
PID_long = tf(Ki_long,[1,0])+tf(Kp_long,1)+tf([Kd_long,0],1); %PID controller
long_dynamics_feedback = feedback(PID_long*long_dynamics,1); %Transfer function for feedback response with PID controller
[y2,t2] = step(long_dynamics_feedback); %Step response with PID controller
S1f = stepinfo(long_dynamics_feedback);
plot(t2,y2);
ess_long2 = y2(end) - 1; %Calculation of steady state error
fprintf('The longitudinal closed loop feedback system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S1f.Overshoot,S1f.SettlingTime,ess_long2)
title('Feedback Control Step Response')

fprintf('\n')

%Linearized Rotational dynamics
num_rot = [.00835,.001745];
den_rot = [1,4.641*(10^-5),.0002839];
rot_dynamics = tf(num_rot,den_rot); %Rotational dynamics transfer function
figure(2)
subplot(2,1,1)
[y,t] = step(rot_dynamics,tstep*1000); %Open loop step response for rotational dynamics
plot(t,y)
S2 = stepinfo(rot_dynamics);
ess_rot = y(end) - 1; %Calculation of steady state error
title('Open Loop Rotational Step Response')
fprintf('The rotational open loop system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S2.Overshoot,S2.SettlingTime,ess_rot)

subplot(2,1,2)
Kp_rot = 75;
Ki_rot = .05; %PID gains for rotational dynamics
Kd_rot = -11;
PID_rot = tf(Ki_rot,[1,0])+tf(Kp_rot,1)+tf([Kd_rot,0],1); %PID controller
rot_dynamics_feedback = feedback(PID_rot*rot_dynamics,1); %Feedback transfer function
[y,t] = step(rot_dynamics_feedback);
S2f = stepinfo(rot_dynamics_feedback);
plot(t,y)
ess_rot2 = y(end) -  1; %Steady state error with feedback
title('Feedback Control Step Response')
fprintf('The rotational closed loop feedback system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S2f.Overshoot,S2f.SettlingTime,ess_rot2)

fprintf('\n')

%Forward mode dynamics
num_fwd = [.0002557,-.02031,.05613];
den_fwd = [1,.4456,27.55,4.316];
fwd_dynamics = tf(num_fwd,den_fwd); %Forward mode transfer function
figure(3)
subplot(2,1,1)
[y,t] = step(fwd_dynamics,tstep/3);
ess_fwd = y(end) - 1; %Open loop steady state error
S3 = stepinfo(fwd_dynamics);
plot(t,y)
title('Open Loop Forward Step Response')
fprintf('The forward open loop system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S3.Overshoot,S3.SettlingTime,ess_fwd)

Kp_fwd = 96;
Ki_fwd = 17; %Closed loop PID gains
Kd_fwd = -67.5;
PID_fwd = tf(Ki_fwd,[1,0])+tf(Kp_fwd,1)+tf([Kd_fwd,0],1); %PID controller
fwd_dynamics_feedback = feedback(PID_fwd*fwd_dynamics,1); %Feedback transfer function
[y2,t2] = step(fwd_dynamics_feedback);
subplot(2,1,2)
plot(t2,y2)
title('Feedback Control Step Response')
S3f = stepinfo(fwd_dynamics_feedback);
ess_fwd2 = y(end) - 1; %Steady state error with feedback
fprintf('The forward feedback loop system has an overshoot of %.2f, a settling time of %.2f s, and a steady state error of %.2f\n',S3f.Overshoot,S3f.SettlingTime,ess_fwd2)

%Plot of open loop root locus for forward mode
figure(4)
rlocus(fwd_dynamics)
title('Root Locus of Forward Mode')


