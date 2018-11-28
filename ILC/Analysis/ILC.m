function [ ] = ILC(sys,Ts)

% Get state space - system already in discrete time
[Ad Bd Cd Dd] = ssdata(sys);

% Set initial condition x0, time range t, pure time delay n0, relative degree r, and matrix sizes N
x0 = 0;
t = 0:Ts:10;
n0 = 0;
r = 1;
N = length(t);

% Define input vector U and reference J
U = 1000*ones(1,N);
Rj = 263.9*ones(N,1) + 10*rand(N,1);

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
jmax = 50;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;

e2k = zeros(jmax,1);
yyy = zeros(jmax,N);

% Run ILC and plot the response for each iteration
figure(2)

for ii = 1:jmax
    Uj = Q*Ujold + L*Ejold;
    Yj = G*Uj;

    Ej = Rj - Yj; Ej(1) = 0;
    Ejold = Ej;
    Ujold = Uj;

    % To visualize response real time
    plot(t,Ej,t,Rj); title(['Iteration: ', num2str(ii)]); ylim([-200 1200]); pause(0.1);

    % yyy(ii,:) = Ej;
    e2k(ii) = Ej' * Ej;
end

% figure(3)
% plot(t,Rj,t,Yj,t,Uj)
% title('ILC - Supervector Format - Iteration 55')
% xlabel('Time (k) [s]')
% ylabel('Amplitude')
% legend('r_k','y_k','u_k')

% figure(4)
% semilogy(1:length(e2k),e2k)
% title('Error as a function of Iteration Index - Semi-log Scale Plot')
% xlabel('Iteration Index')
% ylabel('Sum of Squares of Error')

% figure(5)
% y = (1:jmax)';
% x = t';
% z = yyy;
%
% waterfall(y,x,z')
% xlabel('Itteration')
% ylabel('Time')
% zlabel('Response')
% colormap spring
