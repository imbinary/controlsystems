
%Setup a pid controller

%CL RESPONSE     RISE TIME       OVERSHOOT  SETTLING TIME  S-S ERROR
%Kp              Decrease        Increase   Small Change   Decrease
%Ki              Decrease        Increase   Increase       Eliminate
%Kd              Small Change    Decrease   Decrease       No Change

clc

Kp = 1;
Kd = 0;
Ki = 0;

Kc = 1;     % choose Kc = 1
Tl = 1;     % T for lead compensation
Tg = 0.05;  % T for lag compensation
A = 1;      % alpha 

s=tf('s');


% setup the PID control
K = pid(Kp,Ki,Kd);
P=tf(K);


% find Kv
syms x b;
Px = Kp + Ki/x + Kd*x;              %pid 
GHx = (0.2*x +3.2)/((x+1)*(x+.8));  % plant
Cg = (x+1/Tg)/(x+1/(b*Tg));         % lag

kv = limit((x*Cg*Px*GHx),0);
% set Beta to that value
B = double(solve(kv==5,b));



%setup a lag compensator

Gc = (s+1/Tg)/(s+1/(B*Tg)); % = 1 initially (lag)
Gl = (s+(1*A)/Tl)/(s+1/(Tl)); % = 1 initially (lead)
GHs = (0.2*s +3.2)/((s+1)*(s+.8));

Cg = (x+1/Tg)/(x+1/(B*Tg));     
Cl = (x+(1*A)/Tl)/(x+1/(Tl));    % lead

limit((x*Cg*Px*GHx),0)
Kv = limit((x * Cg * Cl * Px * GHx),x,0);
fprintf('The value of Kv is %s\n',char(Kv));

% show root locus
figure() 
rlocus(P*Kc*Gc*Gl*GHs)
hold on
rlocus(GHs)
sgrid
hold off
figure()
sys = feedback(P * Kc * Gc * Gl * GHs,1);

% show unit step plot
step(sys)
t = 0:.01:100;
figure()
% show unit ramp plot
lsim(sys,t,t)
title('Response to Unit Ramp Input')
sse = abs(1-dcgain(sys));
fprintf('The sse is %f\n',sse);

tf(P * Kc * Gc * Gl * GHs,1)
