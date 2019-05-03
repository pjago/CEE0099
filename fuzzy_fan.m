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

T = 0.1;            %tempo de amostragens
n = 300;            %numero de amostras
o = 3;              %início de amostragem
t = (0:(n-1))*T;    %vetor  de tempo

%% I/O
%tenta conectar com a planta, simula caso aconteça algum erro
z = tf('z', T, 'variable', 'z^-1');
Gz = z^-1*(0.077508 + 0.17161*z^-1)/(1 - 0.8539*z^-1 - 0.02473*z^-2);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[termina, leitura, escrita, planta] = startcom('COM20', Gz);

%% CONFIGURACAO
pasta = planta*'pratica' + ~planta*'teorica';
salvar_em = ['ventilador/' pasta '/fuzzy'];
saturacao = 100;
referencia = 100;
escala = 200;

%ESTADO INCIAL
[r, y, e, u, pwm] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% FUZZY UZI
addpath('.\jsonlab-1.5')
pasta_fuzzy = '.\fuzzy\membership\fan1\';
mfe = membership(loadjson([pasta_fuzzy 'e.json']));
mfde = membership(loadjson([pasta_fuzzy 'de.json']));
mfu = membership(loadjson([pasta_fuzzy 'u.json']));

AND = @(x1, x2) min(x1, x2);
OR = @(x1, x2) max(x1, x2);
IF = @(y, member) centroid_area([y member(y)]);

% AND min
% OR max
% IF centroid
% -e & -de = --u
% -e & +de | 0e & -de = -u
% -e & 0de | +e & 0de | 0e & 0de = 0u
% 0e & +de | +e & -de = +u
% +e & +de = ++u

%% CONTROL LOOP
K = dcgain(Gz);
dr = (escala - referencia)/4;

for k = 3:n
    %MEDIR SENSOR
    time = tic;
    y(k) = leitura();

    %REFERÊNCIA
    r(k) = referencia + dr*(-1*(k>n/5) + 2*(k>2*n/5) - 3*(k>3*n/5) + 4*(k>4*n/5));
    e(k) = r(k) - y(k);

    %CONTROLE
    %fuzzification
    fe = mfe.fx(e(k));
    fde = mfde.fx((y(k) - y(k-1))/T);
    [ne, ze, pe] = fe{:};
    [nde, zde, pde] = fde{:};
    [nnu, nu, zu, pu, ppu] = mfu.fy{:};
    %rule system
    rules = ...
    [
    IF(AND(ne, nde), nnu);
    IF(OR(OR(AND(ne, pde), AND(ne, zde)), AND(ze, nde)), nu);
    IF(AND(ze, zde), zu);
    IF(OR(OR(AND(pe, nde), AND(pe, zde)), AND(ze, pde)), pu);
    IF(AND(pe, pde), ppu);
    ];
    %defuzzification
%     u(k) = sum(rules(:,1).*rules(:,2))/sum(rules(:,2));
    u(k) = (r(k)/K - 50) + sum(rules(:,1).*rules(:,2))/sum(rules(:,2));

    %ATUAÇÃO E SATURAÇÃO
    pwm(k) = min(max(u(k), 0), saturacao);
    
    escrita(pwm(k));
    ping(k) = toc(time);

    %DELAY
    if planta
        while toc(time) < T
        end
    end
end
termina();
fprintf('Tempo: %f segundos\n\n', toc(t0) - toc(time));

%% PLOT & SALVE
fig = plotudo(t(o:k), y, r, e, u, pwm, 0, 0, 0);
if ~exist(salvar_em, 'dir')
    mkdir(salvar_em);
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [salvar_em '/' date];
save([path '.mat'], salvar{:})
saveas(fig, [path '.fig'])
disp(['Plant: ' salvar_em ' Saved at: ' path])
