function [ry, kc, ec] = closed_step (set, COM, Gz) % todo: ???
% closed_step
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'kp', 'kc', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% TUNING

info = lsiminfo(impulse(Gz));
kp = sum(Gz.num{1})/sum(Gz.den{1});
mg = margin(Gz);
drop = 100;
kc = (drop/set - 1)/kp;
for i = 1:100
    drop = min(drop + 0.01*((1/kc + kp)*set/info.Max - drop), 100);
    kc = (drop/set - 1)/kp;
end

[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
dominant = poles(dp);
tau = -Gz.Ts/log(dominant);
mvg = round(2*pi/(wn(dp)*Gz.Ts)) + 1;

%% ESTADO INCIAL

kp = kp*ones(n, 1);
kc = kc*ones(n, 1);
ec = zeros(n, 1);

for j=1:2
    [ry, r, y, e, u, pwm] = deal(zeros(n, 1)); 
    ping = nan(n, 1);
    t0 = tic;

%% LOOP DE CONTROLE
    for k = 1:n
        %LEITURA
        time = tic;
        y(k) = read();

        %REFERENCIA E ERRO
        if j ~= 1
            r(k) = round((1/kc(end) + kp(end))*set);
        elseif k < mvg
            r(k) = round((1/kc(k) + kp(k))*set);
            ry(k) = kp(k)*set;
        else
            ry(k) = ry(k-1) + (y(k) - ry(k-1))/(mvg-1);
            ec(k) = drop - set*(1 + kc(k-1)*kp(k-1));
            kc(k) = kc(k-1) + 0.001*mg*((1 + 0.1*T/tau)*ec(k) - ec(k-1));
            kp(k) = ry(k)/set;
            r(k) = set/kc(k) + ry(k);
        end
        e(k) = r(k) - y(k);

        %CONTROLE
        if j ~= 1
            u(k) = kc(end)*e(k);
        else
            u(k) = kc(k)*e(k);
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

%% PLOT & SAVE

    fig(j) = plotudo(t, y, r, e, u, pwm, 0, 0);
    if isa(stop, 'function_handle')
        pause(mvg*T)
    end
    read();
    
end

if isa(stop, 'function_handle')
    folder = 'closed_step';
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
    saveas(fig(2), [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path 10])
    close(fig(1))
    close(fig(2))
end