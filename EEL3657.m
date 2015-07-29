
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

%%
Ki = 1;
s=tf('s');
GH = (0.2*s +3.2)/((s+1)*(s+.8));

%%
% solve PID control
z=-log(.10)/sqrt(pi^2+log(.10)^2);       % Mp formula
wn=4/(8*z);                             % Wn formula for Ts 8 2% criteria
N=[0 0 wn^2];       
D=[1 2*wn*z wn^2];
p= roots(D);

S1 = p(1);
syms kd kp x
GHs = (0.2*S1 +3.2)/((S1+1)*(S1+.8));  % GH at S1     

%1+P*GHs=0, P = Kp + Kd*s +Ki/s
%Kp + Kd*S1 = -1/GHs-Ki/(S1)
% Ki = 1 chosen value
% use imaginary part to solve Kd
Kd = sym2poly(solve(imag(S1)*kd==imag(-1/GHs-Ki/(S1)),kd));
%use real part to solve Kp
Kp = sym2poly(solve(kp+real(S1)*Kd==real(-1/GHs-Ki/(S1)),kp));
P = Kp+Kd*s+Ki/s;  %combine to create PID control


%%
% find Kv
syms b;
Kc = 1;  % choose Kc = 1 (not used here as reference)
Tg = 0.05; % pick a small value

Px = Kp + Ki/x + Kd*x;              %pid generated above symbolic for solving
Gh = (0.2*x +3.2)/((x+1)*(x+.8));   % plant
c = (x+1/Tg)/(x+1/(b*Tg));          % lag
kv = limit((x*c*Px*Gh),0);
                                    % solve for b and set Beta to that value
B = double(solve(kv==5,b));

%setup a lag compensator

Gc = (s+1/Tg)/(s+1/(B*Tg)); % setup S domain lag controller using values calculated or chosen
Cg = (x+1/Tg)/(x+1/(B*Tg)); % setup symbolic lag controller using values calculated or chosen    

Kv = limit((x * Cg * Px * Gh),x,0); % calculate the real Kv to verify result


%%
% show data
% show root locus
figure() 
rlocus(P * Gc * GH)
sgrid(z,wn)


sys = feedback(P * Gc * GH,1);
figure()
% show unit step plot
step(sys)
t = 0:.01:100;
figure()
% show unit ramp plot
lsim(sys,t,t)
title('Response to Unit Ramp Input')
sse = abs(1-dcgain(sys));
S = stepinfo(sys);

minreal(tf(P * Gc * GH))
fprintf('For the new system shown above the following results. \n');
fprintf('The design and results of PID control\n');
fprintf('Using a Zeta and Wn of (%0.2f, %0.2f)\n',z,wn);
fprintf('The dominant CL pole (S1) is %5.4g + j%5.4g\n', real(S1), imag(S1));
fprintf('The Kp, Ki, and Kd are (%0.2f, %0.2f, %0.2f)\n',Kp,Ki,Kd);
fprintf('The sse for step input is %f\n',sse);
fprintf('The overshoot is %f%%\n',S.Overshoot);
fprintf('The settling time is %0.2f\n',S.SettlingTime);
fprintf('\nThe design and results of Lag compensator\n');
fprintf('The Beta is %0.2f\n',B);
fprintf('The value of Kv is %s\n',char(Kv));
fprintf('The value of sse for ramp input is %f\n',1/sym2poly(Kv));
fprintf('The design and results of PID control\n');


