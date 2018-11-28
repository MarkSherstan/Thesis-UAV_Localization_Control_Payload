function [ ] = ILC(sys,Ts)

% Convert to discrete time using ZOH and find state space
ssd = c2d(sys,Ts,'ZOH');
[Ad Bd Cd Dd] = ssdata(ssd);

% Set up initial conditions and rr vector
x0 = 0;
rr = [3*ones(1,35) 1*ones(1,35) 4*ones(1,35) 2*ones(1,35) 0*ones(1,37)]; % [3 1 4 2 0];
% rr is actually U

figure
lsim(ssd,rr,[],x0)
ylabel('Amplitude')
xlabel('time [s]')

% Find sample time, n0, t, N
t = 0:Ts:1;
n0 = 0;
r = 1;
N = length(t);

% Assume G0 goes to 0

% % Formulate G0
% G0 = zeros(N,1);
% G0(2,1) = Cd' * Ad^(n0+r+1);
%
% for ii = 3:length(G0)
%     G0(ii,1) = Cd' * Ad^(n0+ii);
% end


% Formulate G
Gvec = zeros(N,1);
% Gvec(1,1) = Cd' * Ad^(r-1) * Bd;
% Gvec(2,1) = Cd' * Ad^(r) * Bd;
rvec = (r-1:N-n0-1)';

for ii = 1:length(rvec)
  Apow_vec = Ad ^ rvec(ii);
  Gvec(ii) = [Cd*Apow_vec*Bd];
end

% for ii = 3:length(Gvec)
%     Gvec(ii,1) = Cd' * Ad^(ii-n0-1) * Bd;
% end

G = tril(toeplitz(Gvec));




% Formulate U to be 1x101, solve, and plot
U = [rr 0]';

y = G*U; % y = G0*x0 + G*U;

%%%%%%%%%%%%%%%%%%%%%%%
% Part F
%%%%%%%%%%%%%%%%%%%%%%%
figure(2)
Rj = U;

jmax = 100;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;

e2k = zeros(jmax,1);

for ii = 1:jmax
    Uj = Q*Ujold + L*Ejold;
    Yj = G*Uj; %Yj = G*Uj + G0*x0;

    Ej = Rj - Yj; Ej(1) = 0;
    Ejold = Ej;
    Ujold = Uj;

    % To visualize response real time
    plot(t,Ej,t,Rj); title(['Iteration: ', num2str(ii)]); pause(0.1);

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
