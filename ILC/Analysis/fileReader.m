fieldnames(C)
SYS = pid(C.Kp,C.Ki,C.Kd,C.Tf,Ts)


% https://www.mathworks.com/videos/control-design-with-matlab-and-simulink-90419.html
clear all
close all
clc

load 1000PWM.txt
A = X1000PWM;

time = A(:,1) * 1e-6;
pos = A(:,2); % Measured at gear (multiply by 2 to get pinion)
voltage = A(:,3);
current = A(:,4);
PWM = A(:,5);

% Find the min angle values for rpm calc
localMin = islocalmin(pos,'MinProminence',30);
idx = find(localMin);
deltaTime = time(idx);

for ii = 1:length(deltaTime) - 1
    rps(ii) = deltaTime(ii+1)-deltaTime(ii);
end

rpm = (1/mean(rps))*60;

%Find transfer function
Ts = mean(diff(time));
t = [0:Ts:time(1) time']';
y = [ones(1,length(0:Ts:time(1)))*current(1) current']';
u = [zeros(1,length(0:Ts:time(1))) PWM']';

% figure; plot(t,y); title('y')
% figure; plot(t,u); title('u')


% t = t(1:1800);
% y = y(1:1800); y(621:end) = 263.9360;
% u = u(1:1800);

figure; plot(t,u,t,y); legend('Input-PWM','Output-Current');
%figure; plot(y)

inputu = u;
outputy = y;

% data = iddata(y,u,Ts);
% sys = tfest(data,3);
%
% sys.num
% sys.den


% % Make some figures
% figure(1)
% plot(time,voltage)
% title('Voltage vs Time')
%
% figure(2)
% plot(time,current)
% title('Current vs Time')
%
% figure(3)
% plot(time,pos,time(idx),pos(idx),'*r')
% title('Position vs time')
