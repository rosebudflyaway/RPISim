
function dist = IntervalDistance( minA, maxA, minB, maxB )

    if minA < minB
        dist = minB - maxA;
    else
        dist = minA - maxB;
    end

end

