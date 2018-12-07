function [sysd1,sysd2,sysd3] = stability(tf1,tf2,ss,Ts)

% System 1 is from system identification.
% System 2 is from system ID with feedback from a PD controller.
% System 3 is from simscape linearization.

% Get A B C and D matrices
[A1 B1 C1 D1] = tf2ss(tf1.num{1},tf1.den{1});
[A2 B2 C2 D2] = tf2ss(tf2.num{1},tf2.den{1});
[A3 B3 C3 D3] = ssdata(ss);

% Find transfer function of system 3 - simscape linearization
[num den] = ss2tf(A3,B3,C3,D3);
tf3 = tf(num,den);

% Test see if the systems are controllable.
Co1 = ctrb(A1,B1);
unco1 = length(A1) - rank(Co1);

Co2 = ctrb(A2,B2);
unco2 = length(A2) - rank(Co2);

Co3 = ctrb(A3,B3);
unco3 = length(A3) - rank(Co3);

% Test see if the systems are observable.
Oo1 = obsv(A1,C1);
obsv1 = length(A1) - rank(Oo1);

Oo2 = obsv(A2,C2);
obsv2 = length(A2) - rank(Oo2);

Oo3 = obsv(A3,C3);
obsv3 = length(A3) - rank(Oo3);

% Convert to transfer functions to discrete time using zero order hold
sysd1 = c2d(tf1,Ts,'ZOH');
sysd2 = c2d(tf2,Ts,'ZOH');
sysd3 = c2d(tf3,Ts,'ZOH');

% Find relative degree, how many more poles than zeros?
r1 = length(pole(sysd1)) - length(zero(sysd1));
r2 = length(pole(sysd2)) - length(zero(sysd2));
r3 = length(pole(sysd3)) - length(zero(sysd3));

% Ensure all poles and zeros are within the unit circle
figure(1)
pzmap(sysd1)
title('Pole Zero Map - System Identification')
axis equal

figure(2)
pzmap(sysd2)
title('Pole Zero Map - System Identification (PD Controller)')
axis equal

figure(3)
pzmap(sysd3)
title('Pole Zero Map - Simscape Linearization')
axis equal

% Plot nyquist and ensure -1 is not encircled
figure(4)
nyquist(sysd1)
title('Nyquist Stability - System Identification')

figure(5)
nyquist(sysd2)
title('Nyquist Stability - System Identification (PD Controller)')

figure(6)
nyquist(sysd3)
title('Nyquist Stability - Simscape Linearization')

% Plot bode
figure(7)
bode(sysd1,'k-',sysd2,'k--',sysd3,'k-.')
legend('System Identification','System Identification (PD Controller)','Simscape Linearization','FontSize',16)
set(findall(gcf,'type','line'),'linewidth',2)
