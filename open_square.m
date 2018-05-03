function [Gjw, w] = open_square (set, COM, Gz) % todo: compare with relay
% open_square
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

[absG, phaG, W] = bode(Gz);
Gjw = absG.*exp(1j*phaG*pi/180);
[~, ws] = min(abs(-1 - Gjw)); % most sensibility

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'eps', 'd', 'a', 't', 'y', 'r', 'e', 'u', 'pwm'};

duration = 30; % todo: calculate from tf
d = 20;

eps = -d*imag(Gjw(ws))*4/pi;
a = d*abs(Gjw(ws))*4/pi;

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% TUNING

[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
wc = geomean([wn(dp) W(ws)]);
mvg = round(2*pi/(wc*Gz.Ts));
sup = round(2*pi/(wn(2)*Gz.Ts));
N = 2*pi/(W(ws)*Gz.Ts);
[bb, ba] = butter(1, [1/mvg 1/sup]);
bo = length(bb);
kp = abs(Gjw(ws));

%% ESTADO INCIAL

[r, y, e, u, pwm, saw, w, z] = deal(zeros(n, 1));
ping = nan(n, 1);
t0 = tic;

%% LOOP DE CONTROLE

for k = 1:n
    %LEITURA
    time = tic;
    y(k) = read();
    
    %REFERENCIA E ERRO
    if k < mvg
        w(k) = y(k);
        r(k) = w(k);
        if k >= bo
            z(k) =  bb*w(k:-1:k-bo+1) - ba(2:end)*z(k-1:-1:k-bo+1);
        end
    else
        w(k) = y(k)/mvg^2 + w(k-1)*(2 - 2/mvg) - w(k-2)*(1 - 1/mvg)^2;
        z(k) =  bb*w(k:-1:k-bo+1) - ba(2:end)*z(k-1:-1:k-bo+1);
        r(k) = w(k) - z(k);
    end
    e(k) = r(k) - y(k);
    
    if k < mvg
    elseif mod(k, N) < N/2
        saw(k) = N/2*sawtooth(2*k*pi/N);
    else
        saw(k) = N/2-N/2*sawtooth(2*k*pi/N);
    end
    
    %CONTROLE
    if k < mvg
        u(k) = set;
    elseif mod(k, N) < N/2
        u(k) = set + d;
    else
        u(k) = set - d;
    end

    %SATURACAO
    if u(k) > 100
        pwm(k) = 100;
    elseif u(k) < 0
        pwm(k) = 0;
    else
        pwm(k) = round(u(k));
    end

    %ESCRITA
    write(pwm(k));
    ping(k) = toc(time);

    %DELAY
    if isa(stop, 'function_handle') && T > ping(k) 
        java.lang.Thread.sleep(1000*(T - ping(k)))
    end
end

stop();
fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

%% ANALYSIS

try %cleaning this once
    Gjw = -pi*sqrt(a^2 - eps^2)/(4*d) - 1j*pi*eps/(4*d);
    disp([10 'You wanted d = ' num2str(d) ', eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(W(ws)) ' rad/s' ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw))])
    edg = [0; diff(u - set)/(2*d)];
    edg = edg.*(abs(edg) >= 1)./(abs(edg));
    edg(isnan(edg)) = 0;
    edg((1:end) < mvg) = 0;
    d = mean((u(logical(edg)) - set).*edg(logical(edg)));
    eps = e(logical(edg)).*edg(logical(edg));
    if ~isempty(eps)
        eps(end) = [];
    end
    eps = mean(eps);
    % for measuring a, find the corresponding peak in beetween two edges!
    if ~isempty(find(edg, 1))
        sig = 2*cumsum(edg) - edg(find(edg, 1));
    else
        sig = edg;
    end
    sig(1:find(edg, 1)-1) = 0;
    edg_id = edg.*sig.*(1:length(sig))';
    edg_id(edg_id == 0) = [];
    for i = 1:length(edg_id)-1
        fst = edg_id(i);
        lst = edg_id(i+1)-1;
        a(i, 1) = max(edg(fst)*e(fst:lst));
    end
    if isnan(eps)
        a = NaN;
    else
        a = mean(a);
    end
    w = 2*pi*maxfreq(u(mvg:end))/T;
    Gjw = -pi*sqrt(a^2 - eps^2)/(4*d) - 1j*pi*eps/(4*d);
    disp(['Result was d = ' num2str(d) ', eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(w) ' rad/s' ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw)) 10])
catch ME % todo: find the Black Swan
    eps = NaN;
    a = NaN;
    disp(['Broke! ' ME.message])
end

%% PLOT & SAVE

fig = plotudo(t, y, r, e, u, pwm);
if isa(stop, 'function_handle')
    pause(mvg*T)
end
    
if isa(stop, 'function_handle')
    folder = 'open_square';
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
    session = dir(folder);
    session = session([session.isdir]);
    if length(session) > 2
        folder = [folder '/' session(end).name];
    end
    date = datestr(datetime('now'));
    date(date == '-' | date == ':') = '_';
    path = [folder '/' date];
    save([path '.mat'], toSave{:})
    saveas(fig, [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path 10])
    close(fig)
end