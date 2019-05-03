function [Mu, ts, tr, tp, itae] = step_info(y, u, T, varargin)
    %[Gjw, w] = step_info(y, u, T)
    zero = find(u, 1);
    fim = length(u);
    t = 0:T:(fim-1)*T;
    [ymax, peak] = max(y);
    ymed = median(y);
    sett = fim - find(flip(y) > 1.02*ymed | flip(y) < 0.98*ymed, 1);
    ts = T*(sett - zero);
    rise = find(y > ymed, 1);
    tr = T*(rise - zero);
    Mu = 100*(ymax - ymed)/ymed;
    tp = T*(peak - zero);
    itae = sum(abs(y - ymed).*(t'))*T;
    umax = max(u);
    umed = median(u);
    disp([' Mu = ' num2str(Mu) ', ts = ' num2str(ts) ', tr = ' num2str(tr) ', tp = ' num2str(tp) ', itae = ' num2str(itae) ', uo = ' num2str(u(zero)) ', udc = ' num2str(umed)])
end