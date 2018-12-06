clear all
close all
clc

% Get the experimental data and create a plot. Output Ts and data to make the plot
fileName = '1000PWM';
[Ts t U Y] = fileReader(fileName);

figure(1)
hold on
  plot(t,U,'--k',t,Y,'-k','LineWidth',1.5)
  legend('Input - PWM (\mus)','Output - Current (mA)','FontSize',16,'Location','SouthEast');
  title('Single Input - Single Output Response','FontSize',16)
  xlabel('Time (s)','FontSize',16)
  ylabel('Input / Output Value','FontSize',16)
  ylim([0 1050])
  xlim([0 10])
hold off

% Load the transfer function from system identification (tf1), the PID controller
% and the system identification PD controller transfer function (tf2) the state
% space model from simscape linearization (ss2), and the paramter estimation values
% for the DC motor (non linear simulation)
load('model_tf_ss_motor_data.mat')

% Stability analysis
close all
[sysd1 sysd2 sysd3] = stability(tf1, tf2, ss2, Ts);

% Formulate a distrubance every 2 rotations of the pinion or every 3 seconds
% after t = 1.5 s (initial step response). Add a big distrubance after 1 full
% of the planetary gear 10.8 s after t = 1.5 s or at ~ 12.3 s
t = 0:Ts:17.5;
N = length(t);
disturbance = zeros(N,1);
counter = 1;

for ii = 267:length(disturbance)
  if (counter > 534) & (counter < 534+20)
    disturbance(ii,1) = 50;
  elseif counter == 555;
    counter = 1;
  end
counter = counter + 1;
end

disturbance(2177:2177+150) = 125;

% Plot the disturbance
figure(10);
plot(t,disturbance,'-k','LineWidth',1.5)
xlabel('Time (s)','FontSize',16)
ylabel('Disturbance (mA)','FontSize',16)
title('Periodic Disturbance due to Gear Train','FontSize',16)


% Implment ILC
% linearSimILC(sysd1,disturbance,Ts)
% linearSimILC(sysd2,disturbance,Ts)
% linearSimILC(sysd3,disturbance,Ts)
% nonLinearSimILC
