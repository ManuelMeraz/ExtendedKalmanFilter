function [K, H] = computeKalmanGain(muBar, sigmaBar, landmarks) 

    % Evaluate H at muBar
    H = zeros(3,3);

    for i=1:3
        xDiff = muBar(1,1) - landmarks(i,1);
        yDiff = muBar(2,1) - landmarks(i,2);

        denominator = sqrt(xDiff^2 + yDiff^2);

        H(i,1) = xDiff / denominator;
        H(i,2) = yDiff / denominator;
    end

    Q = eye(3,3) * 0.001;

    K = sigmaBar * H.' * ((H * sigmaBar * H.' + Q)^(-1));
end
