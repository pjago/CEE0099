%simulates Gz by global y, v, k
function yk = readsim(Gz)
    global y pwm k
    yk  = nan(1, size(Gz, 1));
    idv = 1:size(Gz, 2);
    for i=1:size(Gz, 1)
        [NUM, DEM] = tfdata(Gz(i), 'v');
        Nnum = length(DEM);
        Ndem = length(NUM);
        d = totaldelay(Gz(i, at(idv, i)));
        if k > d
            yk(1, i) = -DEM(2:Ndem)*at(y, k-1: -1: k-Ndem+1, i) ... 
                       +NUM(1:Nnum)*at(pwm, k-d-Ndem+Nnum: -1: k-d-Ndem+1, i);
        else
            yk(1, i) = at(y, k-1, i);
        end      
    end
end