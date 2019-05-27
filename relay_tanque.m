clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

% water flow is a function of water pressure!
% imense value of relay identification methods: neglects disturbances
% simple mqnr would consider all output to be due the input
% the relay in another hand, is a very specific kind of input
% so we know exactly at which frequencies to look at the output
% maybe for chaotic, or weirdly non-linear systems this may be false.
% a drone is the perfect place to test this
% another use case is for integrator plants with asymetric inputs
% a challenge may be plants with very long dead time (temperature)

%% CONFIGURAÇÃO
% Adicione o nome de variáveis que queira salvar
salvar_em = 'relay_pi_nivel_valvula_semi';

T = 1;                   %tempo  de amostragem
o = 3;                   %início de amostragem
n = floor(600/T) + o;    %número de amostras
t = (0:(n-1));           %vetor  de tempo

%% I/O
s = tf('s');
z = tf('z', T, 'variable', 'z^-1');
Gz = 0.0083116*T/(z-1);  % relé com histerese (1° artigo)
% Gz = 0.013411*T/(z-1); % relé pic (2° artigo valor final)

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
umax = 9.5; % maximum actuation
umin = 2.0;  % minimum actuation
ref  = 0.0;  % fixed reference

%% RELAY
tune_edge = 3; % non-mpc filters wait this many edges
d = 2.0; % relay amplitude
b = 5.0; % relay bias
eps = 0.05; % important! there is a maximum value of valid eps! 3rd quad.
D = 0.0*d; % disturbance ratio (todo: solve saturation problem!)
% dados calculados
Gss = ss(Gz);
% Gss = ss(0.004245/s); % assume a wrong but close model, then adapt
% Gss = ss(0.016975/s); % it is better to adapt from a good model
% Gss = ss(0.0028/s);
% Gss = ss(0.025/s);
K = Gss.c*Gss.b; %initial model
Ko = K; %this is too lazy TODO cleanup
ns = round((2*eps)/(K*d*T));
nf = 2*ns;
o = o + nf;

% setup
tune = 0;
edge = 0;
semi = [0; 0];
semik = nan(n, 2);
tunek = zeros(n, 1);
upk = nan(n, 1);
deps = nan(n, 1);
up = 1;
eup = 1;
saw = 1;
yid = [0; 0];
rid = [0; 0];
ypk = [0; 0];
rpk = [0; 0];
epk = [0; 0];

%% FILTER
% higher quality: robust to noise. lower quality: robust to disturbances
% on low epsilon it may be useful to increase the quality (precise switch)
% on higher epsilon it may be useful to reduce the quality (fast settling)
QF = 2.0;  %^quality, ^settling time, ^precision

%for QF > 5.0, phase error < 50/QF° for a band the size of the frequency
%for QF = 1.0, it is a fir filter, so prefer this when the noise is low

%calculates comb filter coefficient based on filter quality
ks = o;
af = 1 / cos(pi / (2 * QF)) + cos(pi / (2 * QF));
af = af + sqrt(af^2 - 4);
af = (af - sqrt(af^2 - 4)) / 2;
% af=+abs(af); % this will pass only the odd frequencies
% af=-abs(af); % this will remove only the even frequencies

% similar filters
rf = 0.95;
b0 = 0.5;
am = 0.5;

%% PID
kp=13.71;td=0;ti=12; % PI do leão

%calculates controllers' numerator
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];
%gains
ke = flip(gpid(kp,td,ti));
ky = flip(gpid(kp,td,ti));
kr = flip(gpi(kp,ti));
ku = 1.0;

%% VLMMS
lambda = 0.5; % [0, 1] step update
beta = 0.5; % [0, 1] exponential weighting
pho = 10.0; % [0, 1] leakage update

A = nan(n, 3); % vector of unknown parameters
H = nan(n, 3); % 1st, 3rd, 5th harmonics
mu = zeros(n, 1); % parameters step
gama = zeros(n, 1); % variable leakage factor
ylmms = zeros(n, 1); % model of the signal
elmms = zeros(n, 1); % error signal
Rlmms = zeros(n, 1); % autocorrelation of elmms(k) and elmms(k-1)

% which initial values do I use???

for k = 1:o-1
    mu(k) = -1;
    gama(k) = 0.0;
    A(k, :) = [bode(Gss, pi/(ns*T)) bode(Gss, 3*pi/(ns*T))/3 bode(Gss, 5*pi/(ns*T))/5]*4/pi*d;
end

%% LOOP DE CONTROLE
[e, ei, er, em, ai, am, u, ur, pwm] = deal(zeros(n, 1));
[r, rm, r1, r3, r5, y, yr] = deal(~planta*ref*ones(n, 1));
ping = nan(n, 1);

t0 = tic;

%TODO: make this independent of the model. use of lsim exemplifies that
for k = o:n
    %LEITURA
    time = tic;
    y(k) = leitura();
    
    %REFERÊNCIA
    %step
%     r(k) = 1.0;
    r(k) = 6.0 ...
    + 1.0*(k >= n/3) ... 
    - 1.0*(k >= n/2) ...
    + 0.01*(k - 2*n/3)*(k >= 2*n/3 && k < 5*n/6) ... 
    + (0.995 - 0.01*(k - 5*n/6))*(k >= 5*n/6);
    if k == ceil(n/3) || k == ceil(n/2)
        edge = 0;
        tune = 0;
    end
    %ramp
%     r(k) = 4.0 + 0.01*(k - n/3)*(k > n/3 && k < n/3 + 100/T) + 1.0*(k >= n/3 + 100/T) ...
%          + 0.01*(k - 2*n/3)*(k > 2*n/3 && k < 2*n/3 + 100/T) + 1.0*(k >= 2*n/3 + 100/T);
    %this should be interesting. close the valve and try this ramp
%     r(k) = 2.0 + 0.02*(k - o)*T;
    
    fs = pi/ns; % this is not smooth, maybe use gradient descent!
% F = 1/2*(1+af)*(1 + z^-ns)/(1 + af*z^-ns);
%     rm(k) = -af  * rm(k-ns) + (y(k) + y(k-ns)) * (1  + af) / 2;                          % type 0
% F = 1/4*((3+af) + 2*(1+af)*z^-ns + (af-1)*z^-nf)/(1 + af*z^-ns); 
%     rm(k) = -af  * rm(k-ns) + 1/4*((3 + af)*y(k) + 2*(1 + af)*y(k-ns) + (af - 1)*y(k-nf)); % type 1
% F = (1 + am)/(b0 + 1)*(b0 + z^-M)/(1 + am*z^-M)
%     rm(k) = -am*rm(k-ns) + (1 + am)/(b0 + 1)*(b0*y(k) + y(k-ns));     % allpass
% F = @(m) 1/2*(1 - 2*rf*cos(m*fs) + rf^2)/(1 - cos(m*fs))*(1 - 2*cos(m*fs)*z^-1 + z^-2)/(1 - 2*rf*cos(m*fs)*z^-1 + rf^2*z^-2)
    r1(k) = 2*rf*cos(fs)*r1(k-1) - rf^2*r1(k-2) + 1/2*(1-2*rf*cos(fs)+rf^2)/(1-cos(fs))*(y(k) - 2*cos(fs)*y(k-1) + y(k-2));
    r3(k) = 2*rf*cos(3*fs)*r3(k-1) - rf^2*r3(k-2) + 1/2*(1-2*rf*cos(3*fs)+rf^2)/(1-cos(3*fs))*(r1(k) - 2*cos(3*fs)*r1(k-1) + r1(k-2));
    r5(k) = 2*rf*cos(5*fs)*r5(k-1) - rf^2*r5(k-2) + 1/2*(1-2*rf*cos(5*fs)+rf^2)/(1-cos(5*fs))*(r3(k) - 2*cos(5*fs)*r3(k-1) + r3(k-2));
% type 1, but with MPC. we did it guys, we did it!
    df = (min(ur(k-1) + d, umax) - max(ur(k-1) - d, umin))/2;
    uf = d*[(up + -~up)*ones(ns-saw, 1); (-up + ~up)*ones(ns, 1); (up + -~up)*ones(saw+1, 1)]; % + ur(k-1)
    yf = lsim(Gss, uf, (0:length(uf)-1)*T, y(k)/Gss.c); % if there is no integrator, there should be + ur(k-1)
    yr(k) = 1/4*(3*y(k) + 2*yf(ns+1) - yf(end)); %sem erro de regime, com oscilação
    rm(k) = 1/4*(3*y(k-ns) + 2*y(k) - yf(ns+1)); %noice
    rm(k) = 1/2*(yr(k) + rm(k)); %erro de regime, oscilação precisa
    
    %ERRO
    e(k) = r(k) - y(k);
    er(k) = yr(k) - y(k);
    ei(k) = (rm(k) - rm(k-ns) - y(k) + y(k-ns))/2;
    em(k) = yf(ns+1) - mean(yf(2:end)); % this is a fake er without noise!
        
    H(k, :) = (up - ~up)*[cos(saw*pi/ns) cos(3*saw*pi/ns) cos(5*saw*pi/ns)]; % cos since phase = 90°
    ylmms(k) = H(k, :)*A(k-1, :)';
    elmms(k) = ei(k) - ylmms(k); % point of access
    Rlmms(k) = beta*Rlmms(k-1) + (1 - beta)*elmms(k)*elmms(k-1);
%     mu(k) = lambda*mu(k-1) + gama(k-1)*Rlmms(k-1)^2; % step size
%     gama(k) = gama(k-1) - 2*mu(k-1)*pho*elmms(k-1)*ylmms(k-1); % leakage factor
    gama(k) = 0.0;
    mu(k) = -0.01;
    A(k, :) = (1 - 2*mu(k)*gama(k))*A(k-1, :) + 2*mu(k)*elmms(k)*ylmms(k); % weight vector
    % how does A updates when mu = 0??? leave this out for now
    
    ai(k) = peak2peak(ei(k-nf:k))/2; % this method of amp. estimation can be improved!
    am(k) = peak2peak(em(k-nf:k))/2; % this method of amp. estimation can be improved!
%     am(k) = sqrt(bode(Gss, pi/(ns*T))^2 + bode(Gss, 3*pi/(ns*T))^2 + bode(Gss, 5*pi/(ns*T))^2)*d*4/pi;

    if up %TODO: simplify
        if y(k) <= ypk(2)
            ypk(2) = y(k);
            yid(2) = saw + ns;
        end
        if yr(k) >= rpk(2)
            rpk(2) = yr(k);
            rid(2) = saw + ns;
        end
        epk(2) = max(em(k), epk(2));
    else
        if y(k) >= ypk(1)
            ypk(1) = y(k);
            yid(1) = saw;
        end
        if yr(k) <= rpk(1)
            rpk(1) = yr(k);
            rid(1) = saw;
        end
        epk(1) = min(em(k), epk(1));
    end
    
    %CONTROLE
    if (er(k) >= eps && ~up) || (er(k) <= -eps && up)
        if tune
            ns = saw;
            nf = semi(~up+1) + saw;
        end
        semi(up + 1) = saw;
        edge = edge + 1;
        up = ~up;
        saw = 1;
        ypk(up + 1) = y(k);
        rpk(up + 1) = yr(k);
        epk(up + 1) = em(k);
    else
        saw = saw + 1;
    end
    semik(k,:) = semi;
    tunek(k) = tune;
    upk(k) = up;
    
    %MODEL 100% adaptation
    deps(k) = (K*d*ns*T)/2 - eps + 0.1*(ai(k) - am(k)); %0.1 | 0.15 
    deps(k) = min(max(deps(k), -0.6*eps), 1.1*eps);
    if tune && edge >= tune_edge
        K = 2*(eps + deps(k))/(d*ns*T);
        Gss = ss(K/s); % this is the apparent system. the real K = K*d/df
    end
    
    %RELAY + PID
    ur(k) = ku*ur(k-1) + kr*r(k-1:k) - ky*(tune*yr(k-2:k) + ~tune*y(k-2:k));
    ur(k) = min(max(ur(k), umin + tune*d), umax - tune*d);
    u(k) = ur(k) + tune*(up*d - ~up*d);
    
    if saw == 1 && ~tune % for now I am doing a latch
        % nice entrance condition!
        % this condition is specific for this model. simplify
        tune = abs(yr(k)-r(k)) < d*ns*K*T && (e(k) <= 0 && up || e(k) > 0 && ~up);
        edge = 0;
    end
    
    %SATURAÇÃO
    if planta
        pwm(k) = min(max(u(k), umin), umax);
    else
        pwm(k) = min(max(u(k), umin), umax) - 5.0 + (k > 2*n/3)*D;
    end
    
    %ESCRITA
    escrita(pwm(k));
    ping(k) = toc(time);
        
    if y(k) > 8.0
        break;
    end
    
    %DELAY
    if planta
        while toc(time) < T
        end
    end
end

if planta
    escrita(0.0);
    end_easyport; %termina();
end
fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

K = K*d/df;
Gss = ss(K/s);
tunek = ~~tunek;

%% TEMP
ns = semik(tunek, :);
nf = sum(ns, 2);
idx = sub2ind(size(ns), 1:size(ns, 1), ~upk(tunek)'+1);
ns = ns(idx)';
de = deps(tunek);
Ks = 2*(eps + de)./(d*ns*T);

figure
plot(ns)
figure
plot(Ks)
figure
plot(deps)
figure
plot(e(tunek))
hold on
plot(er(tunek))
plot(ai(tunek))
plot(ei(tunek))
plot(am(tunek))
plot(em(tunek))

%% PLOT & SAVE
[fig, ax1, ax2] = plotudo(t(1:k), y, r, e, u, pwm, 0, 0, 0);
% axes(ax2)
% hold on
% plot(ur, 'k')
% axes(ax1)
% hold on
% plot(yr)
if planta
    pasta = ['pratica/' salvar_em];
else
    pasta = ['teorica/' salvar_em];
end
if ~exist(pasta, 'dir')
    mkdir(pasta);
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [pasta '/' date];
save([path '.mat'], '-regexp', '^(?!(fig|ax1|ax2)$).')
saveas(fig, [path '.fig'])
disp(['Plant: ' pasta ' Saved at: ' path])
