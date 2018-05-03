% TODO: garantir a oscilação, principalmente quando o modelo não for exato
% TODO: check relation between notch quality and transient

clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO

% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm'};

T = 0.05;         %tempo de amostragem
n = 101;          %número de amostras
t = (0:(n-1))*T;  %vetor de tempo

%% I/O

%caso não ache a planta, o programa simula pela função de transferência Gz
Gz = filt([0 0 0.03847], [1 -0.686 -0.2374], 0.05);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM20', Gz);

%% ESTADO INCIAL

w   = 2/4;
Gjw = evalfr(Gz, exp(1j*w*pi));
kp  = dcgain(Gz);
set = 10;
ref = set*kp;
d   = (0.05*100*kp)/(abs(Gjw)*4/pi);
a   = 4*d/pi*abs(Gjw);
eps = -4*d/pi*imag(Gjw);

D  = 0.5;
ko = 2*d/(pi*a*sqrt(1 - (eps/a)^2)); % only exact at the limit eo -> 0
uo = (D - 0.5)*2*d;
eo = uo/ko;
yo = kp*uo;
ro = eo + yo;

%limit cycles
cycle = false(n, 1);
spin = ones(n, 1);

[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% LOOP DE CONTROLE

% ONLY FOR SIMULATION!!!
for k = 1:2/w+1
    y(k) = ref + ro - eps;
    r(k) = ref + ro;
    e(k) = eps;
    u(k) = set - d;
    pwm(k) = u(k);
    write(pwm(k));
end

for k = 2/w+2:n
    %LEITURA
    time = tic;
    y(k) = read();

    %REFERÊNCIA E ERRO
    r(k) = ref + ro;
    e(k) = r(k) - y(k);
    
    %CONTROLE (TODO: INVESTIGATE THE INFLUENCE OF % &&)
    if e(k) >= eps && spin(k-1) == -1
       cycle(k) = true;
       spin(k) = 1;
    elseif e(k) <= -eps && spin(k-1) == 1
       cycle(k) = true;
       spin(k) = -1;
    else
       cycle(k) = cycle(k-1);
       spin(k) = spin(k-1); 
    end
    u(k) = set + d*spin(k);
    
    %SATURAÇÃO
    if u(k) > 100
        pwm(k) = 100;
    elseif u(k) < 0
        pwm(k) = 0;
    else
        pwm(k) = u(k);
    end
    
    %ESCRITA
    write(u(k));
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
