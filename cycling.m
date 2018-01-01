function [ry, sup, inf, kc] = cycling (set, COM, Gz) % todo: ???
% continuos cycling (zigler nichols method)
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;
d = 10; % target oscilation
a = d/margin(Gz)*4/pi;

kp = sum(Gz.num{1})/sum(Gz.den{1});
Gz.TimeUnit = 'seconds';
[kc, ~, w, ~] = margin(Gz);

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'd', 'a', 'w', 'kp', 'kc', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% TUNING

mg = margin(Gz);
[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
dominant = poles(dp);
tau = -Gz.Ts/log(dominant);
mvg = round(2*pi/(wn(dp)*Gz.Ts)) + 1;

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% ESTADO INCIAL

kp = kp*ones(n, 1);
kc = kc*ones(n, 1);
ed = zeros(n, 1);

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
    
    if j == 1
        ry = kp*set;
        sup = (set + d)*ones(n, 1);
        inf = (set - d)*ones(n, 1);
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
            r(k) = (1/kc(1) + kp(end))*set;
        elseif k < mvg
            r(k) = (1/kc(1) + kp(k))*set;
        else
            ry(k) = ry(k-1) + (y(k) - ry(k-1))/(mvg - 1);
            ed(k) = d - (sup(k-1) - inf(k-1))/2;
            if abs(ed(k)) < 0.1*d
                kc(k) = kc(k-1);
            else
                kc(k) = kc(k-1) + 0.01*mg*((1 + 0.01*T/tau)*ed(k) - ed(k-1));
            end
            kp(k) = ry(k)/set;
            r(k) = set/kc(1) + ry(k);
        end

        if k == mvg
            if y(k) > y(k-1)
                r(k) = (set + d)/kc(1) + y(k);
            else
                r(k) = (set - d)/kc(1) + y(k);
            end
        end

        e(k) = r(k) - y(k);

        %CONTROLE
        if k < mvg
            u(k) = set;
        elseif j ~= 1
            u(k) = kc(end)*e(k);
        else
            u(k) = kc(k)*e(k);
        end
        
        if j == 1 && k > mvg + 1
            if u(k) < u(k-1) && u(k-1) >= u(k-2)
                sup(k) = sup(k-1) + (u(k-1) - sup(k-1))/(mvg - 1);
            else
                sup(k) = sup(k-1);
            end
            if u(k) > u(k-1) && u(k-1) <= u(k-2)
                inf(k) = inf(k-1) + (u(k-1) - inf(k-1))/(mvg - 1);
            else
                inf(k) = inf(k-1);
            end
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

disp([10 'You wanted d = ' num2str(d) ', a = ' num2str(a) ', w = ' num2str(w) ' rad/s' ', Gjw = ' num2str(pi*a/(4*d)) ' <' num2str(-pi)])
d = (mean(findpeaks(pwm(mvg:end)-mean(pwm(mvg:end)))) + mean(findpeaks(-pwm(mvg:end)+mean(pwm(mvg:end)))))/2;
a = (mean(findpeaks(e(mvg:end)-mean(e(mvg:end)))) + mean(findpeaks(-e(mvg:end)+mean(e(mvg:end)))))/2;
w = 2*pi*meanfreq(pwm - mean(pwm), 1/T);
disp(['Result was d = ' num2str(d) ', a = ' num2str(a) ', w = ' num2str(w) ' rad/s' ', Gjw = ' num2str(pi*a/(4*d)) ' <' num2str(-pi) 10])

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