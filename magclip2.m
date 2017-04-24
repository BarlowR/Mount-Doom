function res = magclip2(val, val2, magmax)
    maxi = max(abs([val, val2]));
    res = [val val2]./max([maxi/magmax, 1]);
end