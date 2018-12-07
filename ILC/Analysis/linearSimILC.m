function [ ] = linearSimILC(sys,disturbance,Ts)

% Get state space - system is already in discrete time from stability function
[Ad Bd Cd Dd] = ssdata(sys);

% Set initial condition x0, time range t, pure time delay n0, relative degree r,
% and matrix size N
x0 = 0;
t = 0:Ts:17.5;
n0 = 0;
r = 1;
N = length(t);

% Define input vector U and reference J
U = [zeros(1,267) 1000*ones(1,N-267)];
Rj = [zeros(1,267) 263.9*ones(1,N-267)]';

% G0 not formulated as initial condition is 0

% Formulate G
Gvec = zeros(N,1);
rvec = ((r-1):(N-n0-1))';

for ii = 1:length(rvec)
  ApowVec = Ad^rvec(ii);
  Gvec(ii) = Cd*ApowVec*Bd;
end

G = tril(toeplitz(Gvec));

% Set up ILC
jmax = 25;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);
I = eye(N);

Uj = zeros(N,1); Ujold = U';
Ej = zeros(N,1); Ejold = Ej;

e2k = zeros(jmax,1);

%% Set up video writer to record the simulation to a .avi video file.
% v = VideoWriter('simulation.avi');
% v.FrameRate = 5;
% open(v);

% Run ILC and plot the response for each iteration
for ii = 1:jmax
  noise = 15*rand(N,1) - 7.5;

  Uj = Q*Ujold + L*Ejold;
  Yj = G*Uj - (I-G)*(noise - disturbance);

  Ej = Rj - Yj; Ej(1) = 0;
  Ejold = Ej;
  Ujold = Uj;

  plotter(ii,t,Ej,Yj,Uj,Rj,U)
  % frame = getframe(gcf);
  % writeVideo(v,frame);

  e2k(ii) = Ej'*Ej;
end

% close(v);

% Plot the 2- norm error and the results of the final iteration
figure
semilogy(1:length(e2k),e2k)
title('2-norm Error as a function of Iteration Index - Semi-log Scale Plot')
xlabel('Iteration Index, j','interpreter','latex','FontSize',16)
ylabel('2-norm Error $||e(k)^2_2||$','interpreter','latex','FontSize',16)
grid on

figure
subplot(2,1,1);
plot(t,Uj,t,U,'-k','LineWidth',1.5);
title('Input, Uj','FontSize',16);
xlabel('Time (s)','FontSize',16);
ylabel('Input PWM (\mus)','FontSize',16);
ylim([-50 2000]);

subplot(2,1,2);
plot(t,Yj,t,Rj,'-k','LineWidth',1.5);
title('Output, Yj','FontSize',16);
xlabel('Time (s)','FontSize',16);
ylabel('Output Response (mA)','FontSize',16);

end
