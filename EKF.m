% Manuel Meraz
% EECS 270 Robot Algorithms
% Simple EKF

mu = [0; 0; 0]
u = [5; 0];
dt = 0.5;
sigmaNaught = eye(3,3) * 0.001
landmarks = [5, 5;  4, 7; -3, 2];

file = fopen('inputs.txt');
inputs = cell2mat(textscan(file, '%f %f'));
fclose(file);

file = fopen('sensor_readings.txt');
sensorReadings = cell2mat(textscan(file, '%f %f %f'));
fclose(file);

file = fopen('sporadic_sensor_readings.txt');
sporadicSensorReadings = cell2mat(textscan(file, '%f %f %f'));
fclose(file);

for i = 1:size(inputs)

    u = inputs(i,:).'
    z = sensorReadings(i, :).'

    % Prediction
    muBar = motionModel(mu, u, dt);
    sigmaBar = adjustSigma(muBar, u, dt, sigmaNaught);

    % Correction
    [K, H] = computeKalmanGain(muBar, sigmaBar, landmarks);
    mu = correction(muBar, K, z, landmarks)
    sigmaNaught = correctSigma(K, H, sigmaBar)

end

