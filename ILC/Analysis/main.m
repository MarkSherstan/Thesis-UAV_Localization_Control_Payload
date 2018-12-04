clear all
close all

% Get the experimental data and create a plot. Output Ts and data to make the plot
fileName = '1000PWM';
[Ts t U Y] = fileReader(fileName);

figure(1)
hold on
  plot(t,U,'-b',t,Y,'-r','LineWidth',1)
  legend('Input-PWM','Output-Current (mA)','FontSize',16,'Location','SouthEast');
  title('Single Input - Single Output Response','FontSize',16)
  xlabel('Time (s)','FontSize',16)
  ylim([0 1050])
  xlim([0 10])
hold off

% Load the transfer function from system identification (tf1), the state space
% model from simscape linearization (ss2), and the paramter estimation values
% for the DC motor (non linear simulation)
load('model_tf_ss_motor_data.mat')

% Stability analysis
[sysd1 sysd2] = stability(tf1, ss2, Ts);

% Implment ILC
