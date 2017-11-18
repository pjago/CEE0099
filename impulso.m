clc
clear
format shortg
addpath 'src'

global y r t e u pwm k

%% CONFIGURAÇÃO

T = 0.1;        %tempo de amostragem
n = 101;        %número de amostras

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(0.077508 + 0.17161*z^-1)/(1 - 0.8539*z^-1 - 0.02473*z^-2);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[read, write, plant] = startcom(T, 'COM6', '0.0.0.0:3001', Gz);

%% LOOP DE CONTROLE

t = (0:(n-1))*T;
[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%LOOP
for k = 1:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERÊNCIA E ERRO
    r(k) = 82.1;
    e(k) = r(k) - y(k);

    %CONTROLE
    if k == 1
        u(k) = 40;
    end
    
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
    if plant
        while toc(time) < T
        end
    end
end

fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
saverun(plant, ping, t, y, r, e, u, pwm)
write(0);
