
%Setup a pid controller

%CL RESPONSE     RISE TIME       OVERSHOOT  SETTLING TIME  S-S ERROR
%Kp              Decrease        Increase   Small Change   Decrease
%Ki              Decrease        Increase   Increase       Eliminate
%Kd              Small Change    Decrease   Decrease       No Change

Kp = 1;
Kd = 0;
Ki = 0;
K = pid(Kp,Ki,Kd);
k=tf(K);

%setup a lag compensator
Kc = 1;  % choose Kc = 1
Tl = 1;
Tg = 0.05;
B = 1; % 1.25;
A = 1;
s=tf('s');

Gc = (s+1/Tg)/(s+1/(B*Tg)); % = 1 initially lag

Gl = (s+(1*A)/Tl)/(s+1/(Tl)); % = 1 initially lead

GH = (0.2*s +3.2)/((s+1)*(s+.8));

% find Kv
syms x b;
P = Kp + Ki/x + Kd*x;
Cg = (x+1/Tg)/(x+1/(B*Tg));
Cl = (x+(1*A)/Tl)/(x+1/(Tl));
Gh = (0.2*x +3.2)/((x+1)*(x+.8));
c = (x+1/Tg)/(x+1/(b*Tg));
% solve(limit((x*c*P*Gh),0),b)
Kv = limit((x * Cg * Cl * P * Gh),0);
fprintf('The value of Kv is %s\n',char(Kv));


% show root locus
figure() 
rlocus(k*Kc*Gc*Gl*GH)
hold on
rlocus(GH)
sgrid
hold off
figure()
sys = feedback(k * Kc * Gc * Gl * GH,1);

% show unit step plot
step(sys)
t = 0:.01:100;
figure()
% show unit ramp plot
lsim(sys,t,t)
title('Response to Unit Ramp Input')
sse = abs(1-dcgain(sys));
fprintf('The sse is %f\n',sse);

