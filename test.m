
clf                                 % Clear graph on screen.

numg=[.2 3.2];                             % Generate numerator of G(s).
deng=poly([1 .8]);               % Generate denominator of G(s).

G=tf(numg,deng)                     % Create and display G(s).
pos=10;
                                    % Input desired percent overshoot.
z=-log(pos/100)/sqrt(pi^2+[log(pos/100)]^2);
                                    % Calculate damping ratio.
rlocus(G)                           % Plot uncompensated root locus.
sgrid(z,0)                          % Overlay desired percent 
                                    % overshoot line.
title(['Uncompensated Root Locus with ' , num2str(pos),...
'% overshoot Line'])                % Title uncompensated root locus.
K=55
                                    % steady-state error for                                     % unit ramp input.
T=feedback(K*G,1)                   % Create and display T(s).	
step(T)                             % Plot step response of uncompensated 
                                    % system.
title(['Uncompensated System Step Response with ' ,...
    num2str(pos),'% overshoot'])    % Add title to uncompensated step 
                                    % response.

Ts=8;
                                    % Input desired settling time.
b=1
                                    % Input lead compensator zero.
                                    % compensator pole. 
a=3
                                    % Enter test lead compensator pole.
numge=conv(numg,[1 b]);             % Generate numerator of Gc(s)G(s).
denge=conv([1 a],deng);             % Generate denominator 
                                    % of Gc(s)G(s).
Ge=tf(numge,denge);                 % Create Ge(s)=Gc(s)G(s).
wn=4/(Ts*z);                        % Evaluate desired natural 
                                    % frequency.
clf                                 % Clear graph on screen.
rlocus(Ge)                          % Plot compensated root locus with 
                                    % test lead compensator pole.
axis([-10,10,-10,10])               % Change lead-compensated 
                                    % root locus axes.
sgrid(z,wn)                         % Overlay grid on lead-compensated 
                                    % root locus.
title(['Lead-Compensated Root Locus with ' , num2str(pos),...
'% Overshoot Line, Lead Pole at ',...
num2str(-a),' and Required Wn'])    % Add title to lead-compensated  
                                    % rootlocus.

                                    % interactively on the root locus.
'Gc(s)'                             % Display label.
Gc=tf([1 b],[1 a])                  % Display lead compensator.
'Gc(s)G(s)'                         % Display label.
Ge                                  % Display Gc(s)G(s).

s=tf([1 0],1);                      % Create transfer function, 's'.
sGe=s*Ge;                           % Create sGe(s) to evaluate Kv.
sGe=minreal(sGe);                   % Cancel common poles and zeros.
Kv=dcgain(K*sGe)                    % Display lead-compensated Kv.
ess=1/Kv                            % Display lead-compensated steady-
                                    % state error for unit ramp input.
'T(s)'                              % Display label.
T=feedback(K*Ge,1)                  % Create and display lead-compensated 
                                    % T(s).

step(T)                             % Plot step response for lead 
                                    % compensated system.
title(['Lead-Compensated System Step Response with ' ,...
    num2str(pos),'% overshoot'])    % Add title to step response of lead- 
                                    % compensated system.