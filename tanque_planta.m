%1. Right-click the editor's tab and select "Change current folder to..."
%2. Run the code either with Command Window, F5 or Ctrl+Enter
%3. Modify the configuration and repeat from (2)

clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'o', 'k', 'n', 'T'};
subpasta = 'relay';

T = 1;          %tempo de amostragens
n = 600;        %numero de amostras
o = 3;          %início de amostragem
t = (0:(n-1))*T;  %vetor  de tempo

z = tf('z', T, 'variable', 'z^-1');
Gz = 0.01595/(z-1);
Gz.ioDelay = 1;

%% I/O

%tenta conectar com a planta, simula caso aconteça algum erro
try
   throw('simula'); %descomente para simular
    start_easyport;
    pause(1)
    enable_pump(1);
    leitura  = @() read_level;
    escrita = @(duty) write_pump(duty);
    termina = @() end_easyport;
    salvar_pasta   = 'planta';
    leitura();
catch ME
    errors = textscan(ME.message, '%[^\n]', 1);
    disp([errors{end}{:}]);
    leitura  = @() readsim(Gz);
    escrita = @(duty) writesim(duty - 3.7);
    termina = 0;
    salvar_pasta   = 'modelo';
    k = 0;
    escrita(0);
    leitura();
end

%% CONFIGURACAO

%GERAL
salvar_em = [salvar_pasta, 'leao/pid_teste'];
saturacao = 9.5;
referencia = 3;        

%ESTADO INCIAL
[r, rf, rr, y, e, er, u, ur, pwm, f] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%PI
% [Gjw, w] = relay_info(y, u, T)
%  d = 2; eps = 0.21729; eo = -0.00099041; a = 0.21729;
w = 0.1904; Gjw = 0.08533*exp(-1.5708i);
K = real(Gjw*(w*1i)/exp(-1*w*1i));
wn = 0.1; %approx. 40s de tempo de subida
zeta = 0.7; %boa reposta, não muito lento
Kc = 2*zeta*wn/K;
z = wn^2/(K*Kc);
s = tf('s');
Gs = K*exp(-s)/s;
C = pid(Kc*(s + z)/s);
kp = C.Kp;
ti = C.Kp/C.Ki;
td = 0;

% [kp, td, ti] = astron(1.0/0.08533, -1.5708*0.8+1.5708, 0.1904, 0.05);
% kp = 8.78;
% td = 0.0;
% ti = 14.003;

%calcula o numerador do controlador PID
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];

%ganhos
ke = flip(gpid(kp,td,ti));
ky = flip(gpid(kp,td,ti));
kr = flip(gpi(kp,ti));
ku = 1.0;

%% CONTROL LOOP

%LOOP
for k = 3:n
    %MEDIR SENSOR
    time = tic;
    y(k) = leitura();

    %REFERÊNCIA
    r(k) = referencia + 2*(k>n/6) + 2*(k>2*n/6) + 1*(k>3*n/6) - 2*(k>4*n/6) - 2*(k>5*n/6);
    e(k) = r(k) - y(k);

    %CONTROLE
    u(k) = u(k-1) + kr*r(k-1:k) - ky*y(k-2:k);

    %ATUAÇÃO E SATURAÇÃO
    u(k) = min(max(u(k), 0), saturacao); % seria melhor usar um anti-windup
    pwm(k) = u(k);
    
    escrita(pwm(k));
    ping(k) = toc(time);

    %DELAY
    if isa(termina, 'function_handle')
        while toc(time) < T
        end
    end
end
fprintf('Tempo: %f segundos\n\n', toc(t0) - toc(time));

if exist('pump', 'var')
    end_easyport
end

%% PLOT & SALVE
fig = plotudo(t(o:k), y, r, e, u, pwm, 0, 1, 0);
if isa(termina, 'function_handle')
    pasta = ['pratica/' subpasta];
else
    pasta = ['teoria/' subpasta];
end
if ~exist(pasta, 'dir')
    mkdir(pasta);
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [pasta '/' date];
save([path '.mat'], toSave{:})
saveas(fig, [path '.fig'])
disp(['Plant: ' pasta ' Saved at: ' path])
