% Manuel Meraz
% EECS 270 Robot Algorithms
% Simple EKF

mu = [0; 0; 0];
u = [5; 0];
dt = 0.5;
sigmaNaught = eye(3,3) * 0.001;

landmarks = [5, 5;  4, 7; -3, 2];
z = [2; 2; 2];

% Prediction
muBar = motionModel(mu, u, dt);
sigmaBar = adjustSigma(muBar, u, dt, sigmaNaught);

% Correction
[K, H] = computeKalmanGain(muBar, sigmaBar, landmarks);
mu = correction(muBar, K, z, landmarks);
sigmaNaught = correctSigma(K, H, sigmaBar);
