function [Ts t U Y] = fileReader(fileName)

% Load file from different directory
fileNameAndPath = strcat('../Data/',fileName,'.txt');
load(fileNameAndPath)
A = X1000PWM;

% Simplify data loaded in
timeSec = A(:,1) * 1e-6;
pos = A(:,2); % Measured at gear (multiply by 2 to get pinion)
voltage = A(:,3);
current = A(:,4);
PWM = A(:,5);

% Find the min angle values for rpm calc
localMin = islocalmin(pos,'MinProminence',30);
idx = find(localMin);
deltatimeSec = timeSec(idx);

for ii = 1:length(deltatimeSec) - 1
    rps(ii) = deltatimeSec(ii+1)-deltatimeSec(ii);
end

rpm = (1/mean(rps))*60;

% Find sample rate and add intial values
Ts = mean(diff(timeSec));
t = [0:Ts:timeSec(1) timeSec']';
U = [zeros(1,length(0:Ts:timeSec(1))) PWM']';
Y = [zeros(1,length(0:Ts:timeSec(1))) current']'; %ones(1,length(0:Ts:timeSec(1)))*current(1)

% Plot the input and output
figure
plot(t,U,t,Y)
legend('Input-PWM','Output-Current');
title('Single Input Single Output Response(s)')
