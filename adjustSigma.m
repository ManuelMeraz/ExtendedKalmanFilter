function sigmaBar = adjustSigma(muBar, u, dt, sigmaNaught)
    % Jacobian of motion model
    A = [1, 0, - u(1,1) * dt * sin(muBar(3,1));
         0, 1, u(1,1) * dt * cos(muBar(3,1));
         0, 0, 1];

    R = eye(3,3) * 0.01;
    R(3,3) = 0.001;

    sigmaBar = A * sigmaNaught * A.' + R;
end
