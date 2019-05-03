clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO

% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'T'};
subfolder = 'relé_combi';

T = 0.114;          %tempo de amostragem
n =  100;          %número de amostras
t = (0:(n-1))*T;  %vetor de tempo

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(0.09142 + 0.2228*z^-1)/(1 - 0.6893*z^-1 - 0.108*z^-2);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz);

%% ESTADO INCIAL

[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

kp=0.8;td=0.0;ti=1.1;

%calculates controllers' numerator
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];

ke = flip(gpid(kp,td,ti));
ku = 1.0;

%% LOOP DE CONTROLE

for k = 3:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERÊNCIA E ERRO
    r(k) = round(90*(7*T));
    e(k) = r(k) - y(k);

    %CONTROLE
    
    %PID
    u(k) = ku*u(k-1) + ke*e(k-2:k);

    %MQNR
%     if k < n/2
%         u(k) = 50 + 10*(2*rand() - 1);
%     else
%         u(k) = 80 + 10*(2*rand() - 1);
%     end

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
    folder = ['pratica/' subfolder];
else
    folder = ['teoria/' subfolder];
end
if ~exist(folder, 'dir')
    mkdir(folder);
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [folder '/' date];
save([path '.mat'], toSave{:})
saveas(fig, [path '.fig'])
disp(['Plant: ' folder ' Saved at: ' path])