function [sysd1 sysd2] = stability(tf1, ss2, Ts)

% System 1 is from system identification and must be converted to state space.
% System 2 is from simscape linearization. Get A B C and D matrices.
[A1 B1 C1 D1] = tf2ss(tf1.num{1},tf1.den{1});
[A2 B2 C2 D2] = ssdata(ss2);

% Test see if the systems are controllable.
Co1 = ctrb(A1,B1);
unco1 = length(A1) - rank(Co1);

Co2 = ctrb(A2,B2);
unco2 = length(A2) - rank(Co2);

% Convert to transfer functions as required then discrete time
[num den] = ss2tf(A2,B2,C2,D2);
sysd1 = c2d(tf1,Ts,'ZOH');
sysd2 = c2d(tf(num,den),Ts,'ZOH');

% Ensure all poles and zeros are within the unit circle
figure(1);
pzmap(sysd1)
title('Pole Zero Map - System Identification')
axis equal

figure(2);
pzmap(sysd2)
title('Pole Zero Map - Simscape Linearization')
axis equal

% Plot nyquist
figure(3)
nyquist(sysd1)
title('Nyquist Stability - System Identification')

figure(4)
nyquist(sysd2)
title('Nyquist Stability - Simscape Linearization')

% Plot bode
figure(5);
bode(sysd1,sysd2)
legend('System Identification','Simscape Linearization','FontSize',16)
set(findall(gcf,'type','line'),'linewidth',2)
