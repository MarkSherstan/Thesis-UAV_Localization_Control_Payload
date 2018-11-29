function [sysd1 sysd2] = stability(tf1, ss2, Ts)

% System 1 is from system identification and must be converted to state space.
% System 2 is from simscape linearization. Get A B C and D matrices.
[A1 B1 C1 D1] = tf2ss(tf1.num{1},tf1.den{1});
[A2 B2 C2 D2] = ssdata(ss2);

% Find transfer function of system 2 - simscape linearization
[num den] = ss2tf(A2,B2,C2,D2);
tf2 = tf(num,den);

% Relative degree. How many more poles than zeros;
r1 = length(pole(tf1)) - length(zero(tf1));
r2 = length(pole(tf2)) - length(zero(tf2));

% Test see if the systems are controllable.
Co1 = ctrb(A1,B1);
unco1 = length(A1) - rank(Co1);

Co2 = ctrb(A2,B2);
unco2 = length(A2) - rank(Co2);

% Test see if the systems are observable.
Oo1 = obsv(A1,C1);
obsv1 = length(A1) - rank(Oo1);

Oo2 = obsv(A2,C2);
obsv2 = length(A2) - rank(Oo2);

% Convert to transfer functions as required then discrete time
sysd1 = c2d(tf1,Ts,'ZOH');
sysd2 = c2d(tf2,Ts,'ZOH');

% Ensure all poles and zeros are within the unit circle
figure
pzmap(sysd1)
title('Pole Zero Map - System Identification')
axis equal

figure
pzmap(sysd2)
title('Pole Zero Map - Simscape Linearization')
axis equal

% Plot nyquist
figure
nyquist(sysd1)
title('Nyquist Stability - System Identification')

figure
nyquist(sysd2)
title('Nyquist Stability - Simscape Linearization')

% Plot bode
figure
bode(sysd1,sysd2)
legend('System Identification','Simscape Linearization','FontSize',16)
set(findall(gcf,'type','line'),'linewidth',2)
