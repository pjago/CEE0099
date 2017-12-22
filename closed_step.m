function closed_step (reference, varargin)
% closed_step
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO

if nargin > 1
    duration = varargin{1};
else
    duration = 10;
end

% Adicione o nome de variáveis que queira salvar
toSave = {'ping', 'kc', 'kp', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.3;                            %tempo de amostragem
n = round(duration/T) + 1;          %número de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
z = tf('z', T, 'variable', 'z^-1');
% Gz = z^-1*(1.891)/(1 - 0.5564*z^-1 + 0.02444*z^-2); % T = 0.5;
% Gz = z^-1*(1.313)/(1 - 0.5929*z^-1 - 0.0009634*z^-2); % T = 0.4;
Gz = z^-1*(0.8047)/(1 - 0.6103*z^-1 - 0.0588*z^-2); % T = 0.3;
kp = sum(Gz.num{1})/sum(Gz.den{1});

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz);

%% ESTADO INCIAL

kp = kp*ones(n, 1);
kc = (100/reference - 1)./kp;

for j=1:2
    [r, y, e, u, pwm] = deal(zeros(n, 1)); 
    ping = nan(n, 1);
    t0 = tic;

%% LOOP DE CONTROLE
    for k = 1:n
        %LEITURA
        time = tic;
        y(k) = read();

        %REFERÊNCIA E ERRO
        if j ~= 1
            r(k) = (1/KC + KP)*reference;
        elseif k < n/4
            r(k) = (1/kc(k) + kp(k))*reference;
        else
            kp(k) = kp(k-1) + 0.005*sum(reference - kc(k-1)*e(floor(n/4):k-1));
            kc(k) = kc(k-1); % not so sure if this works
            r(k) = (1/kc(k) + kp(k))*reference;
        end
        e(k) = r(k) - y(k);

        %CONTROLE
        if j ~= 1
            u(k) = KC*e(k);
        else
            u(k) = kc(k)*e(k);
        end

        %SATURAÇÃO
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
    KP = mean(kp(floor(n/2):end));
    KC = (100/reference - 1)/KP;
    kc = KC; % todo: maybe reach this incrementaly?
    stop();
    fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
    if sum(ping(1:end-1)' > T)
        disp('In-loop latency is too high! Increase your sampling time.')
    end

%% PLOT & SAVE

    fig(j) = plotudo(t, y, r, e, u, pwm, 0, 0);
    pause(10*T)
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
        folder = [folder '\' session(end).name];
    end
    date = datestr(datetime('now'));
    date(date == '-' | date == ':') = '_';
    path = [folder '/' date];
    save([path '.mat'], toSave{:})
    saveas(fig(2), [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path])
    close(fig(1))
    close(fig(2))
end