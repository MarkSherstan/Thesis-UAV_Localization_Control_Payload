clear all
close all

% Get the data
fileName = '1000PWM';
[Ts t U Y] = fileReader(fileName);

% Transfer function and coeffcients from system identification
num = [5.0861   27.1602];
den = [1.0000   31.9422  101.6955];
tf1 = tf(num,den);

% Load state space model from simscape linearization
load('simscapeLinearModelSS.mat')

% Plot the results of the system identification
figure
hold on
  plot(t,U,'-b',t,Y,'-r','LineWidth',1)
  Ytf = 1000*step(tf1,5);
  plot(t(533:length(Ytf)+532),Ytf,'-k','LineWidth',2)
  legend('Input-PWM','Output-Current (mA)','Linear Transfer Function','FontSize',16,'Location','SouthEast');
  title('Single Input - Single Output Response(s)','FontSize',16)
  xlabel('Time (s)','FontSize',16)
  ylim([0 1050])
  xlim([0 10])
hold off

% Stability analysis
[sysd1 sysd2] = stability(tf1, linsys1, Ts);

% Implment ILC
%ILC(tf1,Ts)

% Estimation result(s):
% RatedV = 9.6297;
% armatureInduc = 7.4126e-06;
% noLoadCurrent = 0.48351;
% noLoadSpeed = 27.2;
% rotorInertia = 9.8946e-06;
% stallTorque = 0.68957;
% supply4noLoadCurrent = 21.841;

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
