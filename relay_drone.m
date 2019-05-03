clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%TODO: use Gz to grab values

%% CONFIGURAÇÃO
% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'T'};
subfolder = 'relay_encubadora_umidade';

T = 0.02;        %tempo  de amostragem
o = 3;            %início de amostragem
n = 2000;          %número de amostras
t = (0:(n-1))*T;  %vetor  de tempo

%% I/O
%caso não ache a planta, o programa simula pela função de transferência Gz
s = tf('s');
z = tf('z', T, 'variable', 'z^-1');
Gs = 23932/(s^3 + 4.7103*s^2 + 5.0726*s);
Gz = c2d(Gs, T);
Gz.Variable = 'z^-1';

% %Fechando a malha
% C1 = (0.0008447*s + 0.01954)/(s + 5.055);
% C2 = 0.001867*s;
% Gf = minreal(C1*Gs/(1 + (C1+C2)*Gs));
% Gz = c2d(Gf, T);
% Gz.Variable = 'z^-1';

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz, 0.00);

%% ESTADO INCIAL
[r, rm, r1, r3, r5, y, e, ei, er, u, ur, pwm, f] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% MISC
umax = +1.0; % maximum pwm
umin = -1.0; % minimum pwm
set = 0.0; % fixed setpoint

%% RELAY
ref = set; % initial reference
edge_comb = 5; % comb waits
d = 0.01;
% d = 1.0;
off = 0.0;
a_max = Inf; % todo: should get from |G(jw)| : w -> min(imag(G(jw)))
eps = 0.5; % important! there is a maximum value for eps!! 3rd quadrant!
D = 0.01;
if D >= 1/2*(a_max/eps - 1) % this should be 1.0 for integrator system
    disp('Disturbance too high, expect a retry!')
end

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
QF = 1.0;  %^quality, ^settling time, ^precision

% there is a maximum and minimum value required for QF!

%for QF > 5.0, phase error > 50/QF° for a band equal the 1st harmonic.
%for QF = 1.0, there is a time delay 

%calculates comb filter coefficient based on filter quality
ks = o;
af = 1 / cos(pi / (2 * QF)) + cos(pi / (2 * QF));
af = af + sqrt(af^2 - 4);
af = (af - sqrt(af^2 - 4)) / 2;
rf = 0.95;    % it should be rf = af^(1/ns). but who knows ns?
a  = af;
% a=+abs(af); % this will pass only the odd frequencies
% a=-abs(af); % this will remove only the even frequencies

%% PID
kp=7.987*10^-4;td=0.3487;ti=0.0;

%calculates controllers' numerator
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];
%gains
ke = flip(gd(kp,td));
ku = 1.0;

%% LOOP DE CONTROLE
for k = o:n
    %LEITURA
    time = tic;
    y(k) = read();
    
    %REFERÊNCIA
    fs = pi/ns; % this is a relay privilege, I should use gradient descent!
    ks = k - ns;
    f(k) = rmax + eps < ymax && rmin - eps > ymin;
% F = 1/2*(1+a)*(1 + z^-ns)/(1 + a*z^-ns);
    rm(k) = -af  * rm(ks) + (y(k) + y(ks)) * (1  + af) / 2;                            % type 0, ood
% F = 1/4*((3+a) + 2*(1+a)*z^-ns) + (a-1)*z^-nf)/(1 + a*z^-ns);
    rm(k) = -af  * rm(ks) + 1/4*((3 + af)*y(k) + 2*(1 + af)*y(ks) + (af - 1)*y(k-nf)); % type 1, odd
% F = @(m) 1/2*(1 - 2*rf*cos(m*fs) + rf^2)/(1 - cos(m*fs))*(1 - 2*cos(m*fs)*z^-1 + z^-2)/(1 - 2*rf*cos(m*fs)*z^-1 + rf^2*z^-2)
    r1(k) = 2*rf*cos(fs)*r1(k-1) - rf^2*r1(k-2) + 1/2*(1-2*rf*cos(fs)+rf^2)/(1-cos(fs))*(y(k) - 2*cos(fs)*y(k-1) + y(k-2));
    r3(k) = 2*rf*cos(3*fs)*r3(k-1) - rf^2*r3(k-2) + 1/2*(1-2*rf*cos(3*fs)+rf^2)/(1-cos(3*fs))*(r1(k) - 2*cos(3*fs)*r1(k-1) + r1(k-2));
    r5(k) = 2*rf*cos(5*fs)*r5(k-1) - rf^2*r5(k-2) + 1/2*(1-2*rf*cos(5*fs)+rf^2)/(1-cos(5*fs))*(r3(k) - 2*cos(5*fs)*r3(k-1) + r3(k-2));
    
    if edge < edge_comb
        r1(k) = set;
        r3(k) = set;
        r5(k) = set;
        rm(k) = set;
        r(k) = set;
    else
        r(k) = rm(k);
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
      % no need to do a retry
      if saw > ns
          if edge >= edge_comb
              retry = retry + 1;
          end
          % this does not work for integrator systems! (?)
          if nf > ns
              nf = 0;
          end
          ns = ns + 1; % up - ymin
          nf = nf + 1; % up - ymax
      end
    end
    
    %RELAY
    er(k) = set - rm(k);
%     ur(k) = ku*ur(k-1) + ke*er(k-2:k);
%     ur(k) = min(max(ur(k), -umax+d), umax-d); % saturando o sinal de controle, fica não linear
    u(k) = off + up*d - ~up*d + (k > n/2)*D*d; % accepts disturbances < d/2*(a_max/eps - 1), but where QF goes?
    %PID
%     u(k) = ku*u(k-1) + ke*e(k-2:k);
    %STEP
%     u(k) = 70;

    %SATURAÇÃO
    pwm(k) = min(max(u(k), umin), umax);
%     pwm(k) = u(k);
    
    %ESCRITA
    write(pwm(k));
    ping(k) = toc(time);
    
    %DELAY
    if isa(stop, 'function_handle')
        while toc(time) < T
        end
    end
end
stop();
fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
% assert(~retry, 'There was a retry! Dont use this for the paper!!!')
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

%% PLOT & SAVE
fig = plotudo(t(o:k), y, r, e, u, pwm, 0, 0, 0);
if isa(stop, 'function_handle')
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