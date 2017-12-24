function relay (set, COM, Gz)
% relay
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

[absG, phaG, W] = bode(Gz);
Gjw = absG.*exp(1j*phaG*pi/180);
[~, ws] = min(abs(-1 - Gjw));
kp = sum(Gz.num{1})/sum(Gz.den{1});

duration = 10;
d = 10;
eps = -(4*d)*imag(Gjw(ws))/pi;
a = sqrt((4/pi*d*real(Gjw(ws)))^2 - eps^2);

% Adicione o nome de varieveis que queira salvar
toSave = {'ping', 'd', 'eps', 'a', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.03;                            %tempo de amostragem
n = round(duration/T) + 1;          %numero de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom(COM, Gz);

%% TUNING

[wn,~,poles] = damp(Gz);
[~, dp] = min(abs(abs(poles) - 1));
dominant = poles(dp);
tau = -Gz.Ts/log(dominant);
mvg = round(2*pi/(wn(dp)*T));

%% ESTADO INCIAL

kp = kp*ones(n, 1);
ep = zeros(n, 1);
ee = zeros(n, 1);

for j=1:2
    % on first run, measure and control eps, by tuning r
    
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
            r(k) = round(kp(end)*set);
        elseif k <= mvg
            r(k) = round(kp(k)*set);
        elseif k < n/4
            ep(k) = y(k-1) - kp(k-1)*set;
            ee(k) = 0;
            kp(k) = kp(k-1) + 0.003*((1 + 10*T/tau)*ep(k) - ep(k-1));
            r(k) = round(kp(k)*set);
        else
            ep(k) = y(k-1) - kp(k-1)*set;
            ee(k) = ee(k-1) + (1 - 1/mvg)*y(k) - y(k-1) + (1/mvg)*y(k-mvg);
            kp(k) = kp(k-1) - 0.001*((1 + 2*T/tau)*ee(k) - ee(k-1));
            r(k) = round(kp(k)*set);
        end
                
        if (k == ceil(n/4))
            r(k) = r(k) + 2*eps;
        end
        
        e(k) = r(k) - y(k);

        %CONTROLE
        if j ~= 1 || k > n/4
            if e(k) >= eps
                u(k) = set + d;
            elseif e(k) < -eps
                u(k) = set - d;
            else
                u(k) = u(k-1);
            end
        else
            u(k) = set;
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
    
    stop();
    fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
    if sum(ping(1:end-1)' > T)
        disp('In-loop latency is too high! Increase your sampling time.')
    end

%% PLOT & SAVE

    fig(j) = plotudo(t, y, r, e, u, pwm, 0, 0);
    pause(10*T)
%     read();
    
end

disp([10 'You wanted eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(W(ws)) ' rad'])    
edg = [0; diff(pwm - set)/(2*d)];
eps = mean(e(logical(edg)).*edg(logical(edg)));
a = peak2peak(e(floor(n/2):end))/2;
wr = meanfreq(pwm - mean(pwm))/T;
disp(['Result was eps = ' num2str(eps) ', a = ' num2str(a) ', w = ' num2str(wr) ' rad' 10])

if isa(stop, 'function_handle')
    folder = 'relay';
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