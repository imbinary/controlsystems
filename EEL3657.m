
%Setup a pid controller

%CL RESPONSE     RISE TIME       OVERSHOOT  SETTLING TIME  S-S ERROR
%Kp              Decrease        Increase   Small Change   Decrease
%Ki              Decrease        Increase   Increase       Eliminate
%Kd              Small Change    Decrease   Decrease       No Change

Kp = 1;
Kd = 0;
Ki = 0;
K = pid(Kp,Ki,Kd);

%setup a lag compensator
Kc = 1;  % choose Kc = 1
Tl = 1;
Tg = 0.05;
B = 1; % 1.25;
s=tf('s');

Gc = Kc*(s+1/Tg)/(s+1/(B*Tg)); % = 1 initially

GH = (0.2*s +3.2)/((s+1)*(s+.8));

% find Kv
syms x;
P = Kp + Ki/x + Kd*x;
C= (x+1/Tl)/(x+1/(B*Tl));
Gh = (0.2*x +3.2)/((x+1)*(x+.8));
Kv = limit((x*C*P*Gh),0);
fprintf('The value of Kv is %s\n',char(Kv));
% show root locus
%figure() 
%rlocus(GH)
%sgrid

figure()
sys = feedback(K * Gc * GH,1);

% show unit step plot
subplot(211), step(sys)
t = 0:.01:100;

% show unit ramp plot
subplot(212), lsim(sys,t,t)
title('Response to Unit Ramp Input')
sse = abs(1-dcgain(sys));
fprintf('The sse is %f\n',sse);

