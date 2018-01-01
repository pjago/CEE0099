clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURACAO

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 't', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.147456;      %tempo de amostragem
n = 101;           %numero de amostras
t = (0:(n-1))*T;   %vetor de tempo

%% ESTADO INCIAL

[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

kc = 1;
ti = 1;
td = T;

%% I/O

%caso nao ache a planta, o programa simula pela funcao de transferencia Gz
Gz = filt([0 0.1 0.05], [1 -1.1 0.2125], T);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('/dev/ttyUSB0', Gz);

%% LOOP DE CONTROLE

for k = 3:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERENCIA E ERRO
    
    % Setpoint emergência
    r(k) = round(80*7*T);
     
%     % Reduz atuação de pico
%     if k < 15
%         r(k) = round(30*7*T);
%     else
%         r(k) = round(80*7*T);
%     end
    
%     % Soft Start
%     if k < 7
%         r(k) = round(30*7*T) + (k-3)/4*round(50*7*T);
%     else
%         r(k) = round(80*7*T);
%     end

    e(k) = r(k) - y(k);

    %CONTROLE
    
%     if k == 3
%        u(k) = 100;
%     end

%     u(k) = 60;

    u(k) = u(k-1) + kc*(e(k) - e(k-1)) + kc/ti*(e(k) + e(k-1))/2*T ...
                  - kc*td*(y(k) - 2*y(k-1) + y(k-2))/T;

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

fig = plotudo(t, y, r, e, u, pwm, 0, 0);

if isa(stop, 'function_handle')
    folder = 'pratica';
else
    folder = 'teoria';
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