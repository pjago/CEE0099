function open_noise (set, COM, Gz)
% open_noise
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;
REF = round(set*sum(Gz.num{1})/sum(Gz.den{1}));
d = 10;

% Adicione o nome de varieveis que queira salvar
toSave = {'ping', 'd', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% ESTADO INCIAL

[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% LOOP DE CONTROLE

for k = 1:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERENCIA E ERRO
    r(k) = REF;
    e(k) = r(k) - y(k);

    %CONTROLE
    u(k) = set + d*(2*rand() - 1);

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
    if isa(stop, 'function_handle')
        while toc(time) < T
        end
    end
end

stop();
fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

%% PLOT & SAVE

fig = plotudo(t, y, r, e, u, pwm, 0, 0);
pause(10*T)
   
if isa(stop, 'function_handle')
    folder = 'open_noise';
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
    saveas(fig(1), [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path 10])
    close(fig)
end