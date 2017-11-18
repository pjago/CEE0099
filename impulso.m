clc
clear
format shortg
addpath 'src'

global y r t e u pwm k

%% CONFIGURA��O

T = 0.1;        %tempo de amostragem
n = 101;        %n�mero de amostras

%% I/O

%caso n�o ache a planta, o programa simula pela fun��o de transfer�ncia Gz
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

    %REFER�NCIA E ERRO
    r(k) = 82.1;
    e(k) = r(k) - y(k);

    %CONTROLE
    if k == 1
        u(k) = 40;
    end
    
    %SATURA��O
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

fprintf('Dura��o: %f seconds\n', toc(t0) - toc(time));
saverun(plant, ping, t, y, r, e, u, pwm)
write(0);
