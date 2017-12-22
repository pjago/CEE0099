function open_noise (reference, varargin)
% noise
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO

d = 10;

if nargin > 1
    duration = varargin{1};
else
    duration = 10;
end

% Adicione o nome de variáveis que queira salvar
toSave = {'ping', 'd', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.3;                            %tempo de amostragem
n = round(duration/T) + 1;          %número de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(1.891)/(1 - 0.5564*z^-1 + 0.02444*z^-2); % T = 0.5;
% Gz = z^-1*(1.313)/(1 - 0.5929*z^-1 - 0.0009634*z^-2); % T = 0.4;
% Gz = z^-1*(0.8047)/(1 - 0.6103*z^-1 - 0.0588*z^-2); % T = 0.3;
REF = reference*sum(Gz.num{1})/sum(Gz.den{1});

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz);

%% ESTADO INCIAL

for j=1:1
    [r, y, e, u, pwm] = deal(zeros(n, 1)); 
    ping = nan(n, 1);
    t0 = tic;

%% LOOP DE CONTROLE
    for k = 1:n
        %LEITURA
        time = tic;
        y(k) = read();

        %REFERÊNCIA E ERRO
        r(k) = REF;
        e(k) = r(k) - y(k);

        %CONTROLE
        u(k) = reference + d*(2*rand() - 1);

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
    folder = 'open_noise';
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
    saveas(fig(1), [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path])
    close(fig(1))
end