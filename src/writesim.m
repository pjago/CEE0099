%writes to readsim target by global u, k
%initial call must have k = 0
function writesim(value)
    global v k
    v(k, :) = value;
end