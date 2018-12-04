% Get state space data - system already in discrete time.
[Ad Bd Cd Dd] = ssdata(sysd2);

% Set initial condition x0, time range t, pure time delay n0, relative degree r,
% and matrix sizes N from length of t.
x0 = 0;
t = 0:Ts:17.5;
n0 = 0;
r = 1;
N = length(t);

% Define input vector U and reference J. Step at 1.5s
Uref = [zeros(1,267) 1000*ones(1,N-267)];
Rj = [zeros(1,267) 263.9*ones(1,N-267)]';

% G0 not formulated as initial conditions are 0

% Formulate G
Gvec = zeros(N,1);
rvec = ((r-1):(N-n0-1))';

for ii = 1:length(rvec)
  ApowVec = Ad^rvec(ii);
  Gvec(ii) = Cd*ApowVec*Bd;
end

G = tril(toeplitz(Gvec));

% Set up ILC variables
jmax = 25;
l0 = 0.95;
q0 = 1;

L = l0 * eye(N,N);
Q = q0 * eye(N,N);
I = eye(N);

Uj = zeros(N,1); Ujold = Uref';
Ej = zeros(N,1); Ejold = Ej;

% Run ILC and plot the response for each iteration
figure(2)

for ii = 1:jmax
  Uj = Q*Ujold + L*Ejold;

  % Requirment for simulink (needs time stamp)
  U = [t' Uj];

<<<<<<< HEAD
  simOut = sim('nonLinearILC');
=======
  simOut = sim('nonLinear');
>>>>>>> 3ca4aea3291633778e83b4c43353a0f165beac99

  Yj = Y.Data + disturbance;

  Ej = Rj - Yj; Ej(1) = 0;
  Ejold = Ej;
  Ujold = Uj;

  plotter(ii,t,Ej,Yj,Uj,Rj,Uref)
end
