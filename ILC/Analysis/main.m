clear all
close all

% Get the data
fileName = '1000PWM';
[Ts t U Y] = fileReader(fileName);

% % Original Estimates of motor parameters and validation results
%
% armatureInduc = 12e-6;
% stallTorque = 0.755;
% noLoadSpeed = 42;
% RatedV = 5;
% noLoadCurrent = 0.21;
% supply4noLoadCurrent = 4.78;
% rotorInertia = 0.0002;
%
% validation = [t Y./1000];

% Estimation result(s):
RatedV = 9.6297;
armatureInduc = 7.4126e-06;
noLoadCurrent = 0.48351;
noLoadSpeed = 27.2;
rotorInertia = 9.8946e-06;
stallTorque = 0.68957;
supply4noLoadCurrent = 21.841;

% Transfer function and coeffcients from system identification
num = [5.0861   27.1602];
den = [1.0000   31.9422  101.6955];

tf1 = tf(num,den);

% Stability analysis
pzmap(num,den)
roots(den) % Find poles
roots(num)
step(num,den)
bode(tf1)

% Implment ILC
ILC(tf1,Ts)
