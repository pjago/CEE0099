function open_step (set, COM, Gz) % todo: return open loop analysis
% open_step
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;
REF = set*sum(Gz.num{1})/sum(Gz.den{1});


% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem  
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% TUNING

[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
mvg = round(2*pi/(wn(dp)*Gz.Ts)) + 1;

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
    u(k) = set;

    %SATURACAO
    if u(k) > 100
        pwm(k) = 100;
    elseif u(k) < 0
        pwm(k) = 0;
    else
        pwm(k) = u(k);
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

%% PLOT & SAVE

fig = plotudo(t, y, r, e, u, pwm, 0, 0);
if isa(stop, 'function_handle')
    pause(mvg*T)
end

if isa(stop, 'function_handle')
    folder = 'open_step';
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