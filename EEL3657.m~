
%PID control
% CL RESPONSE     RISE TIME       OVERSHOOT  SETTLING TIME  S-S ERROR
% Kp              Decrease        Increase   Small Change   Decrease
% Ki              Decrease        Increase   Increase       Eliminate
% Kd              Small Change    Decrease   Decrease       No Change
% lag-lead control
%lag  remove sse
%lead improve transient response

clear all
close all
clc

Kp = 1;     % 1 for a no change PID
Kd = 0;     % 0 for a no change PID
Ki = 1;     % 0 for a no change PID

Kc = 1;     % choose Kc = 1 20
Tl = 1;     % T for lead compensation
Tg = 0.02;  % T for lag compensation
A = 1;      % alpha (1 for no change to initial system)
B = 1;
Z = .5;
wn=1;

% stuff for S domain
s=tf('s');
GHs = (0.2*s +3.2)/((s+1)*(s+.8)); % plant
Gc = (s+1/Tg)/(s+1/(B*Tg)); % = 1 initially (lag)
Gl = (s+(1*A)/Tl)/(s+1/(Tl)); % = 1 initially (lead)
% setup the PID control

% stuff for solving 
syms x b ki;
Px = Kp + ki/x + Kd*x;              %pid 
GHx = (0.2*x +3.2)/((x+1)*(x+.8));  % plant

%%
% PID design
tmp = GHx/(1+GHx * Px);
pretty(tmp)

Px = Kp + Ki/x + Kd*x;              %pid 
P = pid(Kp,Ki,Kd)


%%
% design a lag controller pick Kc=1
% find Kv
Cg = (x+1/Tg)/(x+1/(b*Tg));         % lag
% set Beta to that value so Kv=5
%limit((x * GHx),x,0)
if limit((x * Cg * Px * GHx),x,0) == 0
    B = 1;
else
    B = double(solve(limit((x * Cg * Px * GHx),x,0)==5,b));
end

%setup the lag compensator using B found above
Cg = (x+1/Tg)/(x+1/(B*Tg));     % lag
limit((x * Cg * Px * GHx),0)
Kv = limit((x * Cg * Px * GHx),x,0);




%%
% Design a lead controller
% setup the lead compensator using A found above

Cl = (x+(1*A)/Tl)/(x+1/(Tl));    % lead

limit((x * Cg * Px * GHx),0)
Kl = limit((x * Cg * Cl * Px * GHx),x,0);

%%
% show data
% show root locus
figure() 
rlocus(P * Kc * Gc * Gl * GHs)
hold on
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
S = stepinfo(sys);

tf(P * Kc * Gc * Gl * GHs)
fprintf('For the new system shown above the following results. \n');
fprintf('The sse is %f\n',sse);
fprintf('The overshoot is %f%%\n',S.Overshoot);
fprintf('The settling time is %f\n',S.SettlingTime);
fprintf('The value of Kv is %s\n',char(Kv));

