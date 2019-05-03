function [Gjw, w, d, a, eps] = relay_info(y, u, T, varargin)
    %[Gjw, w] = relay_info(y, u, T)
    try %cleaning this once
    n = length(y);
    settled = (1:n > 0)'; % not good!!
    u = u - mean(u(settled));
    spin = round(2*u/(peak2peak(u))); % no longer true!!
%     spin = round(2*(u - mean(u))/(peak2peak(u)));
    e = mean(y(settled))-y;
    edg = settled.*[0; diff(spin)]/2;
    bet = logical(edg) | logical(circshift(edg, -1));
%     figure
%     hold on
%     plot(spin)
%     plot(edg)
    if nargin < 4
      d = mean((u(logical(edg)) - mean(u)).*edg(logical(edg)));
    else
      d = varargin{1};
    end
    epsaux = e(bet).*(edg(bet) + circshift(edg(bet), -1));
%     figure
%     hold on
%     plot(edg(bet) + circshift(edg(bet), -1))
%     plot(e(bet))
    if ~isempty(epsaux) % todo: if find 1 cycle
        eo = mean(e(settled));
        eps = max(0, mean(epsaux));
    end
    % for measuring a, find the corresponding peak in beetween two edges!
    edg_id = sign(edg).*sign(spin.*settled).*(1:n)';
    edg_id(edg_id == 0) = [];
    edg_delta = [];
    aaux = [];
    for ide = 1:length(edg_id)-1
        fst = edg_id(ide);
        lst = edg_id(ide+1)-1;
        edg_delta = [edg_delta; lst - fst + 1];
        aaux(ide, 1) = max(sign(edg(fst))*e(fst:lst));
    end
    if ~isempty(aaux) % todo: if find 1 cycle
        a = median(aaux);
    else
        a = peak2peak(e(round(end/2):end))/2;
    end
    
    if ~mod(length(edg_delta), 2)
        edg_delta(end) = [];
    end
    edg_delta = qshift(edg_delta, [1, 1]);
    w = 2*pi/(mean(edg_delta)*T);
%     w = 2*pi*medfreq(u-mean(u), 1/T) %which one is less worst?
%     w = pi*maxfreq(u)/T;
    Gjw = -pi/(4*d)*sqrt(a^2 - eps^2) - 1j*pi/(4*d)*eps;
%     Gjw = -1/(2*d/(pi*a)*(sqrt(1 - ((eps + eo)/a)^2) + sqrt(1 - ((eps - eo)/a)^2)) - 1j*4*d*eps/(pi*a^2));
    %Lh = (asin(eps/a)/w); % this is what the delay was supposed to be
    %L = 2*T*finddelay(y, u) - Lh; % the rest is lag
    %L = T; % let's hard code here
    disp([' d = ' num2str(d) ', eps = ' num2str(eps) ', eo = ' num2str(eo) ', a = ' num2str(a) ',' 10 ' w = ' num2str(w) ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw)) 10])
    catch ME % todo: find the Black Swan
        eps = NaN;
        a = NaN;
        Gjw = NaN;
        w = NaN;
        disp(['Broke! ' ME.message])
    end
end