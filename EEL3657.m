
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

<<<<<<< HEAD
Kp = 1;     % 1 for a no change PID
Kd = 0;     % 0 for a no change PID
Ki = 0;     % 0 for a no change PID

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
tmp = feedback(GHs,1);
figure();
SO = stepinfo(tmp);
step(tmp);
hold on
KP = 1;
KI = 0;
KD = 0;
for KP = 1:.5:3
    for KI = 1:.5:3
        for KD = 1:.5:3
            P = pid(KP,KI,KD);
            tmp = feedback(P * GHs,1);
            S = stepinfo(tmp);
            sse = abs(1-dcgain(tmp));
            i=1;
            if S.Overshoot < 10 && S.Overshoot <= SO.Overshoot && sse < 0.01 && S.SettlingTime < 8 && S.SettlingTime <= SO.SettlingTime
                SO = S;
                Ki=KI;
                Kp=KP;
                Kd=KD;
                %Names(i,3)=num2str(Kp);
                step(tmp);
                i=i+1;
            end
        end
    end
end
%Names
title('Candidate PID controls')
hold off
Px = Kp + Ki/x + Kd*x;              %pid 
P = pid(Kp,Ki,Kd);


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
%B=1
%setup the lag compensator using B found above
Cg = (x+1/Tg)/(x+1/(B*Tg));     % lag
limit((x * Cg * Px * GHx),0)
Kv = limit((x * Cg * Px * GHx),x,0);




%%
% Design a lead controller
% setup the lead compensator using A found above

Cl = (x+(1*A)/Tl)/(x+1/(Tl));    % lead
=======
Kp = 1;
Kd = 1;
Ki = 1;
s=tf('s');
GH = (0.2*s +3.2)/((s+1)*(s+.8));

%%
% solve PID control
z=-log(.10)/sqrt(pi^2+log(.10)^2);
wn=4/(8*z);
N=[0 0 wn^2];
D=[1 2*wn*z wn^2];
p= roots(D);

S1 = p(1);
syms kd kp x
GHs = (0.2*S1 +3.2)/((S1+1)*(S1+.8));
%1+P*GHs=0, P = Kp + Kd*s +Ki/s
%Kp + Kd*S1 = -1/GHs-Ki/(S1)
% Ki = .1 small
% use imaginary part to solve Kd
Kd = sym2poly(solve(imag(S1)*kd==imag(-1/GHs-Ki/(S1)),kd));
%use real part to solve Kp
Kp = sym2poly(solve(kp+real(S1)*Kd==real(-1/GHs-Ki/(S1)),kp));
P = Kp+Kd*s+Ki/s;


%%
% find Kv
syms b;
Kc = 1;  % choose Kc = 1
Tg = 0.05;

Px = Kp + Ki/x + Kd*x;           %pid generated above
Gh = (0.2*x +3.2)/((x+1)*(x+.8));  % plant
c = (x+1/Tg)/(x+1/(b*Tg));   % lag
kv = limit((x*c*Px*Gh),0);
% set Beta to that value
B = double(solve(kv==5,b));

%setup a lag compensator

%B = 1.25; % 1.25;
Gc = (s+1/Tg)/(s+1/(B*Tg)); % = 1 initially (lag)

Cg = (x+1/Tg)/(x+1/(B*Tg));     

limit((x*c*Px*Gh),0)
Kv = limit((x * Cg * Px * Gh),x,0);
fprintf('The value of Kv is %s\n',char(Kv));
>>>>>>> 11e4b6008e11ac72eac206a0c42d01a7a410ce2d

limit((x * Cg * Px * GHx),0)
Kl = limit((x * Cg * Cl * Px * GHx),x,0);

%%
% show data
% show root locus
figure() 
<<<<<<< HEAD
rlocus(P * Kc * Gc * Gl * GHs)
hold on
=======
rlocus(P*Gc*GH)
>>>>>>> 11e4b6008e11ac72eac206a0c42d01a7a410ce2d
sgrid
figure()
<<<<<<< HEAD

sys = feedback(P * Kc * Gc * Gl * GHs,1);
=======
sys = feedback(P * Gc * GH,1);
>>>>>>> 11e4b6008e11ac72eac206a0c42d01a7a410ce2d

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
fprintf('The Kp, Ki, and Kd are (%0.2f, %0.2f, %0.2f)\n',Kp,Ki,Kd);
fprintf('The Beta is %f\n',B);
fprintf('The overshoot is %f%%\n',S.Overshoot);
fprintf('The settling time is %f\n',S.SettlingTime);
fprintf('The value of Kv is %s\n',char(Kv));

<<<<<<< HEAD
=======
minreal(tf(P * Gc * GH,1))
>>>>>>> 11e4b6008e11ac72eac206a0c42d01a7a410ce2d
