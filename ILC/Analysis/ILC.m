function [ ] = ILC(sys,Ts)

% Get state space - system already in discrete time
[Ad Bd Cd Dd] = ssdata(sys);

% Set up initial conditions and input vector U
x0 = 0;
U = [1000*ones(1,354) 1200*ones(1,354) 1000*ones(1,354) 1200*ones(1,354) 0*ones(1,356) 0];

% Run linear simulation
figure(1)
lsim(sys,U,[],x0)
ylabel('Amplitude')
xlabel('time [s]')

% Set time period, n0, r and N
t = 0:Ts:10;
n0 = 0;
r = 1;
N = length(t);

% Assume G0 goes to 0

% % Formulate G0
% G0 = zeros(N,1);
% rvec = ((n0+r):N)';
%
% %G0(2,1) = Cd' * Ad^(n0+r+1);
%
% for ii = 1:length(rvec)
%   Apow_vec = Ad^rvec(ii);
%   %G0(ii) = Cd*Apow_vec;
% end

% Formulate G
Gvec = zeros(N,1);
% Gvec(1,1) = Cd' * Ad^(r-1) * Bd;
% Gvec(2,1) = Cd' * Ad^(r) * Bd;
rvec = ((r-1):(N-n0-1))';

for ii = 1:length(rvec)
  Apow_vec = Ad^rvec(ii);
  Gvec(ii) = Cd*Apow_vec*Bd;
end

% for ii = 3:length(Gvec)
%     Gvec(ii,1) = Cd' * Ad^(ii-n0-1) * Bd;
% end

G = tril(toeplitz(Gvec));


% Formulate U to be 1x101, solve, and plot
%U = [rr 0]';
U = U';

y = G*U; % y = G0*x0 + G*U;

%%%%%%%%%%%%%%%%%%%%%%%
% Part F
%%%%%%%%%%%%%%%%%%%%%%%
figure(2)
Rj = U;

jmax = 150;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;

e2k = zeros(jmax,1);
yyy = zeros(jmax,N);

for ii = 1:jmax
    Uj = Q*Ujold + L*Ejold;
    Yj = G*Uj; %Yj = G*Uj + G0*x0;

    Ej = Rj - Yj; Ej(1) = 0;
    Ejold = Ej;
    Ujold = Uj;

    % To visualize response real time
    %plot(t,Ej,t,Rj); title(['Iteration: ', num2str(ii)]); pause(0.1);
    yyy(ii,:) = Ej;
    e2k(ii) = Ej' * Ej;
end

figure(3)
plot(t,Rj,t,Yj,t,Uj)
title('ILC - Supervector Format - Iteration 55')
xlabel('Time (k) [s]')
ylabel('Amplitude')
legend('r_k','y_k','u_k')

figure(4)
semilogy(1:length(e2k),e2k)
title('Error as a function of Iteration Index - Semi-log Scale Plot')
xlabel('Iteration Index')
ylabel('Sum of Squares of Error')

figure(5)
y = (1:jmax)';
x = t';
z = yyy;

waterfall(y,x,z')
xlabel('Itteration')
ylabel('Time')
zlabel('Response')
colormap spring
