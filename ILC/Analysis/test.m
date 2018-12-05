[A2 B2 C2 D2] = ssdata(linsys1);
% Find transfer function of system 2 - simscape linearization
[num den] = ss2tf(A2,B2,C2,D2);
tf2 = tf(num,den);

[Cpid,info] = pidtune(tf2,'PID')


% 0.1 respone with 0.9 transient
Kp = 4.3027;
Ki = 102.6293;
Kd = 0;
k = 1/0.2639; % Negative feddback gain


C = pid(Kp,Ki,Kd)

sys = feedback(C*tf1,k);

step(sys)

sysd = c2d(sys,Ts,'ZOH')
