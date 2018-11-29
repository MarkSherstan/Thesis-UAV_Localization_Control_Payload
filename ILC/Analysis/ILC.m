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
jmax = 50;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);
I = eye(N);

noise = 20*rand(N,1);
disturbance = [zeros(1,1000) 100*ones(1,73) zeros(1,700)]';

figure(1); plot(disturbance)

Uj = zeros(N,1); Ujold = Uj;
Ej = zeros(N,1); Ejold = Ej;

e2k = zeros(jmax,1);
yyy = zeros(jmax,N);

% Run ILC and plot the response for each iteration
figure(2)

for ii = 1:jmax
    Uj = Q*Ujold + L*Ejold;
    Yj = G*Uj - (I - G)*(noise + disturbance);

    Ej = Rj - Yj; Ej(1) = 0;
    Ejold = Ej;
    Ujold = Uj;

    noise = 20*rand(N,1);

    % To visualize response real time
    subplot(1,3,1); plot(t,Ej,t,Rj); title(['Ej Iteration: ', num2str(ii)]); ylim([-50 600]); pause(0.1);
    subplot(1,3,2); plot(t,Yj,t,Rj); title(['Yj Iteration: ', num2str(ii)]); ylim([-50 600]); pause(0.1);
    subplot(1,3,3); plot(t,Uj,t,Rj); title(['Uj Iteration: ', num2str(ii)]); ylim([-50 1100]); pause(0.1);

    yyy(ii,:) = Ej;
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


figure(5)
y = (1:jmax)';
x = t';
z = yyy;

waterfall(y,x,z')
xlabel('Itteration')
ylabel('Time')
zlabel('Error')
colormap spring
