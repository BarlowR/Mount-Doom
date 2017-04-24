function res = clip(val, min, max)
    if val > max
        res = max
    elseif val < min
        res = min
    else
        res = val
    end
end