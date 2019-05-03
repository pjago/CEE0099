clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%% CONFIGURAÇÃO
% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'T'};
subfolder = 'relay_tanque_nivel';

T = 1;            %tempo  de amostragem
o = 3;            %início de amostragem
n = 1200 + o;      %número de amostras
t = (0:(n-1));    %vetor  de tempo

%% I/O
s = tf('s');
z = tf('z', T, 'variable', 'z^-1');
Gz = 0.0081048/(z-1);

planta = 0; % variável deve chavear leitura e escrita da planta

if planta
    start_easyport;
    pause(3)
    enable_pump(1);
    leitura  = @() read_level;
    escrita = @(duty) write_pump(duty);
    termina = @() end_easyport;
    leitura();
else
    leitura  = @() readsim(Gz);
    escrita = @(duty) writesim(duty);
    termina = 0;
    k = 0;
    escrita(0);
    leitura();
end

%% MISC
umax = 10.0; % maximum actuation
umin = 0.0;  % minimum actuation
ref  = 4.0;  % fixed reference

%% RELAY
edge_comb = 5; % comb waits this many edges
d = 4.0; % relay amplitude
off = 4.0; % relay offset
a_max = 0.5*d; % should get from |G(jw)| : w -> min(imag(G(jw)))
eps = 0.1; % important! there is a maximum value for eps!! 3rd quadrant!
D = 0.0; % disturbance ratio at k > n/2

if D > 1/2*(a_max/eps - 1)
    disp('Disturbance too high, expect a retry!')
end
% dados calculados
edge = 0;
semi = [0; 0];
nf = 1;
ns = 1;
up = 1;
saw = 1;
retry = 0;
ymax = ref;
ymin = ref;
rmax = ref;
rmin = ref;

%% FILTER
% higher quality: sensible to noise. lower quality: robust to disturbances
% on low epsilon it may be useful to increase the quality (precise switch)
% on higher epsilon it may be useful to reduce the quality (fast settling)
QF = 8.0;  %^quality, ^settling time, ^precision

% there is a maximum and minimum value required for QF!

%for QF > 5.0, phase error < 50/QF° for a band the size of the frequency
%for QF = 1.0, there is a time delay 

%calculates comb filter coefficient based on filter quality
ks = o;
af = 1 / cos(pi / (2 * QF)) + cos(pi / (2 * QF));
af = af + sqrt(af^2 - 4);
af = (af - sqrt(af^2 - 4)) / 2;
rf = 0.90; % it should be rf = af^(1/ns). but ns is not known yet
mu = 1e-5; % gradient descent

% af=+abs(af); % this will pass only the odd frequencies
% af=-abs(af); % this will remove only the even frequencies

%% PID
kp=7.987*10^-4;td=0.3487;ti=0.0; % atualizar valores!!!

%calculates controllers' numerator
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];
%gains
ke = flip(gd(kp,td));
ku = 1.0;

%% LOOP DE CONTROLE
[r, rm, r1, r3, r5, beta, y, e, ei, er, u, ur, pwm] = deal(zeros(n, 1)); 
f = pi*ones(n, 1);
ping = nan(n, 1);
t0 = tic;

for k = o:n
    %LEITURA
    time = tic;
    y(k) = leitura();
    
    %REFERÊNCIA
    % adaptando o notch fundamental
%     f(k-1) = f(k-1)*7;
%     beta(k) = 1/2*(2*rf*sin(f(k-1))*(1-cos(f(k-1))) - (1-2*rf*cos(f(k-1))+rf^2)*sin(f(k-1)))/(1-cos(f(k-1)))^2*(y(k) - 2*cos(f(k-1))*y(k-1) + y(k-2)) + 1/2*(1-2*rf*cos(f(k-1))+rf^2)/(1-cos(f(k-1)))*2*sin(f(k-1))*y(k-1) - 2*rf*sin(f(k-1))*r1(k-1) + 2*rf*cos(f(k-1))*beta(k-1) - rf^2*beta(k-2);
%     beta(k) = ...
%         1/2*(2*rf*sin(f(k-1))*(1-cos(f(k-1))) - (1-2*rf*cos(f(k-1))+rf^2)*sin(f(k-1)))/(1-cos(f(k-1)))^2*(r3(k) - 2*cos(f(k-1))*r3(k-1) + r3(k-2)) +... 
%         +1/2*(1-2*rf*cos(f(k-1))+rf^2)/(1-cos(f(k-1)))*(2*sin(f(k-1))*r3(k-1)) + ...
%         -2*rf*sin(f(k-1))*r5(k-1) + 2*rf*cos(f(k-1))*beta(k-1) - rf^2*beta(k-2);
%     f(k-1) = f(k-1)/7;
%     if edge < edge_comb
%         f(k) = pi/ns;
%     else
%         f(k) = f(k-1) + 2*mu*r1(k-1)*beta(k);
%     end
%     f(k) = max(0, min(pi, f(k)));
%     fs = f(k);
    fs = pi/ns; % this is not smooth, should use gradient descent!
    ks = k - ns;
% F = 1/2*(1+af)*(1 + z^-ns)/(1 + af*z^-ns);
%     rm(k) = -af  * rm(ks) + (y(k) + y(ks)) * (1  + af) / 2;                            % type 0, ood
% F = 1/4*((3+af) + 2*(1+af)*z^-ns + (af-1)*z^-nf)/(1 + af*z^-ns);
    rm(k) = -af  * rm(ks) + 1/4*((3 + af)*y(k) + 2*(1 + af)*y(ks) + (af - 1)*y(k-nf)); % type 1, odd
% F = @(m) 1/2*(1 - 2*rf*cos(m*fs) + rf^2)/(1 - cos(m*fs))*(1 - 2*cos(m*fs)*z^-1 + z^-2)/(1 - 2*rf*cos(m*fs)*z^-1 + rf^2*z^-2)
    r1(k) = 2*rf*cos(fs)*r1(k-1) - rf^2*r1(k-2) + 1/2*(1-2*rf*cos(fs)+rf^2)/(1-cos(fs))*(y(k) - 2*cos(fs)*y(k-1) + y(k-2));
    r3(k) = 2*rf*cos(3*fs)*r3(k-1) - rf^2*r3(k-2) + 1/2*(1-2*rf*cos(3*fs)+rf^2)/(1-cos(3*fs))*(r1(k) - 2*cos(3*fs)*r1(k-1) + r1(k-2));
    r5(k) = 2*rf*cos(5*fs)*r5(k-1) - rf^2*r5(k-2) + 1/2*(1-2*rf*cos(5*fs)+rf^2)/(1-cos(5*fs))*(r3(k) - 2*cos(5*fs)*r3(k-1) + r3(k-2));
    
    if edge < edge_comb
        r1(k) = ref;
        r3(k) = ref;
        r5(k) = ref;
        rm(k) = ref;
        r(k) = ref;
    else
        r(k) = rm(k);
%         r(k) = r5(k);
    end
    
    if ~up
        ymax = max(y(k), ymax);
        rmax = max(r(k), rmax);
    else
        ymin = min(y(k), ymin);
        rmin = min(r(k), rmin);
    end
    
    %ERRO
    e(k) = r(k) - y(k);
    ei(k) = ei(k-1) + T*(e(k) + e(k-1))/2;
    er(k) = ref - rm(k);
    
    %CONTROLE
    if (e(k) >= eps && ~up) || (e(k) <= -eps && up)
      semi(up + 1) = saw;
      ns = saw;
      nf = semi(~up+1) + saw;
      edge = edge + 1;
      saw = 1;
      if up
        ymax = y(k);
        rmax = r(k);
      else
        ymin = y(k);
        rmin = r(k);
      end
      up = ~up;
    else
      saw = saw + 1;
      if saw > ns
          % attention: this retry mechanism is too complex to be on the
          % paper. in the future simplify it and test which frequencies
          % need it. it seems that for low eps there is no need.
          % this is in accordance with the < d/2*(a_max/eps - 1) hipothesis
          % so for the paper, assert retry = 0 !!!
          if edge >= edge_comb
              retry = retry + 1;
          end
          if nf > ns
              nf = 0;
          end
          ns = ns + 1; % up - ymin
          nf = nf + 1; % up - ymax
      end
    end
    
    %RELAY
    u(k) = off + up*d - ~up*d + (k > n/2)*D*d;
    %PID
%     u(k) = ku*u(k-1) + ke*e(k-2:k);
    %STEP
%     u(k) = 70;

    %SATURAÇÃO
    if planta
        pwm(k) = min(max(u(k), umin), umax);
    else
        pwm(k) = min(max(u(k), umin), umax) - 3.5;
    end
%     pwm(k) = u(k);
    
    %ESCRITA
    escrita(pwm(k));
    ping(k) = toc(time);
    
    %DELAY
    if planta
        while toc(time) < T
        end
    end
end
termina();
fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
% assert(~retry, 'There was a retry! Dont use this for the paper!!!')
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

%% PLOT & SAVE
% o = 36;
% k = 450+o;
% o = 218;
% k = 469;
fig = plotudo(t(o:k), y, r, e, u, pwm, 0, 0, 0);
if planta
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