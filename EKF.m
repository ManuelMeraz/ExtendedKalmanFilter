% Manuel Meraz
% EECS 270 Robot Algorithms
% Simple EKF

mu = [0; 0; 0]
u = [0.5; 0]
dt = 0.5
sigmaNaught = eye(3,3) * 0.001

landmarks = [1, 1; 2, 2;, 3, 3]

% Prediction
muBar = motionModel(mu, u, dt)
sigmaBar = adjustSigma(muBar, u, dt, sigmaNaught)

% Correction
K = computeKalmanGain(muBar, sigmaBar, landmarks)
