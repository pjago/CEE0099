clc
clear
format shortg
addpath 'src'

global y r t e u v k

%% CONFIGURA��O

T = 0.1;        %tempo de amostragem
n = 101;        %n�mero de amostras

%% I/O

%fun��o de transfer�ncia para simula��o, caso n�o ache planta
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(0.077508 + 0.17161*z^-1)/(1 - 0.8539*z^-1 - 0.02473*z^-2);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[read, write, plant] = startcom(T, 'COM6', '0.0.0.0:3001', Gz);

%% LOOP DE CONTROLE

t = (0:(n-1))*T;
[r, y, e, u, v] = deal(zeros(n, 1)); 
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
    u(k) = 40;
    
    %SATURA��O
    if u(k) > 100
        v(k) = 100;
    elseif u(k) < 0
        v(k) = 0;
    else
        v(k) = u(k);
    end
    
    %ESCRITA
    write(v(k));
    ping(k) = toc(time);
    
    %DELAY
    if plant
        while toc(time) < T
        end
    end
end

fprintf('Dura��o: %f seconds\n', toc(t0) - toc(time));
saverun(plant, ping, t, y, r, e, u, v)
write(0);
