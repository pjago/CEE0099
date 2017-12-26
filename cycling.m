function cycling (set, COM, Gz)
% continuos cycling (zigler nichols method)
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

duration = 30;
d = 10; % target oscilation
eps = 0;
a = 4/pi*d/margin(Gz);

kp = sum(Gz.num{1})/sum(Gz.den{1});
Gz.TimeUnit = 'seconds';
[kc, ~, Wc, ~] = margin(Gz);

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'kp', 'kc', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = Gz.Ts;                          %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% TUNING

[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
dominant = poles(dp);
tau = -Gz.Ts/log(dominant);
mvg = ceil(2*pi/(wn(dp)))*3;

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
        elseif k <= mvg
            r(k) = round((1/kc(k) + kp(k))*set);
            ry(k) = kp(k)*set;
        else
            ry(k) = ry(k-1) + (y(k) - ry(k-1))/mvg;
            ep(k) = ry(k-1) - kp(k-1)*set;
            ec(k) = 2*d - abs(u(k-1) - u(k-2));
            kp(k) = kp(k-1) + 0.003*((1 + 5*T/tau)*ep(k) - ep(k-1));
            kc(k) = kc(k-1) + 0.001*((1 + 2*T/tau)*ec(k) - ec(k-1));
            r(k) = round((1/kc(k) + kp(k))*set);
        end
                
        if k == mvg + 1
            r(k) = (set + d)/kc(end) + y(k);
        end
        e(k) = r(k) - y(k);

        %CONTROLE
        if k <= mvg
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

disp([10 'You wanted d = ' num2str(d) ', eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(Wc) ' rad/s' ', Gjw = ' num2str(pi*sqrt(a^2 + 2*eps^2)/(4*d)) ' <' num2str(atan2(-pi*eps/(4*d), -pi*sqrt(a^2 + eps^2)/(4*d)))])
fst = mvg + 1;
d = (mean(findpeaks(pwm(fst:end)-mean(pwm(fst:end)))) + mean(findpeaks(-pwm(fst:end)+mean(pwm(fst:end)))))/2;
a = (mean(findpeaks(e(fst:end)-mean(e(fst:end)))) + mean(findpeaks(-e(fst:end)+mean(e(fst:end)))))/2;
wr = 2*pi*meanfreq(pwm - mean(pwm), 1/T);
disp(['Result was d = ' num2str(d) ', eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(wr) ' rad/s' ', Gjw = ' num2str(pi*sqrt(a^2 + 2*eps^2)/(4*d)) ' <' num2str(atan2(-pi*eps/(4*d), -pi*sqrt(a^2 + eps^2)/(4*d))) 10])

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