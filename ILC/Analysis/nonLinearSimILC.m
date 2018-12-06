% Set time range t, pure time delay n0, relative degree r,
% and matrix sizes N from length of t.
t = 0:Ts:17.5;
n0 = 0;
r = 1;
N = length(t);

% Define input vector U and reference R. Step at 1.5s
Uref = [zeros(1,267) 1000*ones(1,N-267)];
Rj = [zeros(1,267) 263.9*ones(1,N-267)]';

% Set up ILC variables
jmax = 25;
l0 = 0.3;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);
I = eye(N);

Uj = zeros(N,1); Ujold = Uref';
Ej = zeros(N,1); Ejold = Ej;

% Run ILC and plot the response for each iteration
figure(2)

for ii = 1:jmax
  noise = 15*rand(N,1) - 7.5;

  Uj = Q*Ujold + L*Ejold;

  % Requirment for simulink (needs time stamp)
  U = [t' Uj];

  simOut = sim('nonLinearModel');

  Yj = Y.Data + disturbance;

  Ej = Rj - Yj; Ej(1) = 0;
  Ejold = Ej;
  Ujold = Uj;

  plotter(ii,t,Ej,Yj,Uj,Rj,Uref)

  e2k(ii) = Ej'*Ej;
end

% close all
%
% figure
% semilogy(1:length(e2k),e2k)
% %title('Error as a function of Iteration Index - Semi-log Scale Plot')
% xlabel('Iteration Index, j','interpreter','latex','FontSize',16)
% ylabel('2-norm Error $||e(k)^2_2||$','interpreter','latex','FontSize',16)
% grid on
%
% figure
% subplot(2,1,1);
% plot(t,Uj,t,Uref,'-k','LineWidth',1.5);
% title('Input, Uj','FontSize',16);
% xlabel('Time (s)','FontSize',16);
% ylabel('Input PWM (\mus)','FontSize',16);
% ylim([-50 2000]);
%
% subplot(2,1,2);
% plot(t,Yj,t,Rj,'-k','LineWidth',1.5);
% title('Output, Yj','FontSize',16);
% xlabel('Time (s)','FontSize',16);
% ylabel('Output Response (mA)','FontSize',16);
