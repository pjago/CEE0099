%1. Right-click the editor's tab and select "Change current folder to..."
%2. Run the code either with Command Window, F5 or Ctrl+Enter
%3. Modify the configuration and repeat from (2)

clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

% Adicione o nome de variáveis que queira salvar
salvar = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'o', 'k', 'n', 'T'};

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
    planta   = 'pratica';
    leitura();
catch ME
    errors = textscan(ME.message, '%[^\n]', 1);
    disp([errors{end}{:}]);
    leitura  = @() readsim(Gz);
    escrita = @(duty) writesim(duty - 3.7);
    termina = 0;
    planta   = 'teoria';
    k = 0;
    escrita(0);
    leitura();
end

%% CONFIGURACAO
salvar_em = [planta, 'pedro/fuzzy'];
saturacao = 9.5;
referencia = 3;

%ESTADO INCIAL
[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% FUZZY UZI @todo

% regras
% cada combinação de entrada é realizada pelo: mínimo
% a fuzzificação de cada entrada é combinada pelas regras (x -> y)
% a aplicação de uma regra em uma função é através do: centro de massa (y)
% a ativação de uma função por múltiplas regras é decidida pelo: máximo (y) (inferência)
% a combinação de múltiplas funções é realizada pelo: centro de massa (y -> x) (defuzificação)
% -e, -de = --u
% -e, +de = -u
% 0e, -de = -u
% -e, 0de = 0u
% +e, 0de = 0u
% 0e, +de = +u
% +e, -de = +u
% +e, +de = ++u

%rule system
%defuzzification

%% CONTROL LOOP
for k = 3:n
    %MEDIR SENSOR
    time = tic;
    y(k) = leitura();

    %REFERÊNCIA
    r(k) = referencia + 2*(k>n/6) + 2*(k>2*n/6) + 1*(k>3*n/6) - 2*(k>4*n/6) - 2*(k>5*n/6);
    e(k) = r(k) - y(k);

    %CONTROLE
    u(k) = 5;

    %ATUAÇÃO E SATURAÇÃO
    pwm(k) = min(max(u(k), 0), saturacao);
    
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
if ~exist(salvar_em, 'dir')
    mkdir(salvar_em);
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [salvar_em '/' date];
save([path '.mat'], salvar{:})
saveas(fig, [path '.fig'])
disp(['Plant: ' salvar_em ' Saved at: ' path])
