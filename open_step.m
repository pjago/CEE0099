function open_step (reference, varargin)
% open_step
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO

if nargin > 1
    duration = varargin{1};
else
    duration = 10;
end

% Adicione o nome de variáveis que queira salvar
toSave = {'ping', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.3;                            %tempo de amostragem
n = round(duration/T) + 1;          %número de amostras
t = (0:(n-1))*T;                    %vetor de tempo

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(0.8047)/(1 - 0.6103*z^-1 - 0.0588*z^-2);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz);

%% ESTADO INCIAL

[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% LOOP DE CONTROLE

for k = 1:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERÊNCIA E ERRO
    r(k) = reference;
    e(k) = r(k) - y(k);

    %CONTROLE
    u(k) = reference;

    %SATURAÇÃO
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

fig = plotudo(t, y, r, e, u, pwm, 0, 0);

if isa(stop, 'function_handle')
    folder = 'open_step';
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
    saveas(fig, [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path])
    pause(10*T)
    close(fig)
end