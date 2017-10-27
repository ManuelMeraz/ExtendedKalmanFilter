function sigmaNaught = correctSigma(K, H, sigmaBar)
    sigmaNaught = (eye(3,3) - K * H) * sigmaBar;
end
