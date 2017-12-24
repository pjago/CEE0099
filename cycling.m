function cycling (set, COM, Gz)
% continuos cycling (zigler nichols method)
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;
d = 5; % target oscilation
kp = sum(Gz.num{1})/sum(Gz.den{1});
[kc, ~, ~, ~] = margin(Gz);

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'kp', 'kc', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% TUNING

[~,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
dominant = poles(dp);
tau = -Gz.Ts/log(dominant);

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% ESTADO INCIAL

kp = kp*ones(n, 1);
kc = kc*ones(n, 1);
ep = zeros(n, 1);
ec = zeros(n, 1);

for j=1:2
    
    if j == 1
        duration = 4*duration;
        n = round(duration/T) + 1;
        t = (0:(n-1))*T;
    else
        duration = duration/4;
        n = round(duration/T) + 1;
        t = (0:(n-1))*T;
    end
    
    [r, y, e, u, pwm] = deal(zeros(n, 1)); 
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
        elseif k < n/6
            r(k) = round((1/kc(k) + kp(k))*set);
        elseif k < n/3
            ep(k) = y(k-1) - kp(k-1)*set;
            ec(k) = 2*d - abs(u(k-1) - u(k-2));
            kp(k) = kp(k-1) + 0.001*((1 + 10*T/tau)*ep(k) - ep(k-1));
            kc(k) = kc(k-1);
            r(k) = round((1/kc(k) + kp(k))*set);
        else 
            ep(k) = y(k-1) - kp(k-1)*set;
            ec(k) = 2*d - abs(u(k-1) - u(k-2));
            kp(k) = kp(k-1);
            kc(k) = kc(k-1) + 0.001*((1 + 8*T/tau)*ec(k) - ec(k-1));
            r(k) = r(k-1);
        end
        
        if (k == floor(n/3)-1)
            r(k) = r(k) - kp(k)*d;
        end
        
        e(k) = r(k) - y(k);

        %CONTROLE
        if k < n/6
            u(k) = set;
        elseif j ~= 1
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
        if isa(stop, 'function_handle')
            while toc(time) < T
            end
        end
    end
    
    if j == 1
        kc(end + 1) = mean(kc);
    end
    
    stop();
    fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
    if sum(ping(1:end-1)' > T)
        disp('In-loop latency is too high! Increase your sampling time.')
    end

%% PLOT & SAVE

    fig(j) = plotudo(t, y, r, e, u, pwm, 0, 0);
    pause(10*T)
    read();
    
end

if isa(stop, 'function_handle')
    folder = 'cycling';
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