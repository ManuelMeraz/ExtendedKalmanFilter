function mu = correction(muBar, K, z, landmarks)
    for i=1:3
        h(i,1) = distance(muBar, landmarks(i,:).');
    end

    ' inside correction'
    mu = muBar + K * (z - h)
    ' inside correction'
end

function d = distance(p1, p2) 
    xDiff = p1(1,1) - p2(1,1);
    yDiff = p1(2,1) - p2(2,1);
    
    d = sqrt(xDiff^2 + yDiff^2);
end

