clc
clear
format shortg
addpath(genpath('src'))

global t y r e u pwm k

%TODO: use Gz to grab values

%% CONFIGURAÇÃO
% Adicione o nome de variáveis que queira salvar
toSave = {'t', 'y', 'r', 'e', 'u', 'pwm', 'ping', 'T'};
subfolder = 'relay_combi';

T = 0.02;        %tempo  de amostragem
o = 3;            %início de amostragem
n = 2051;          %número de amostras
t = (0:(n-1))*T;  %vetor  de tempo

%% I/O
%caso não ache a planta, o programa simula pela função de transferência Gz
s = tf('s');
z = tf('z', T, 'variable', 'z^-1');
Gs = 23932/(s^3 + 4.7103*s^2 + 5.0726*s);
Gz = c2d(Gs, 0.02);
Gz.Variable = 'z^-1';

C1 = (0.0008447*s + 0.01954)/(s + 5.055);
C2 = 0.001867*s;
Gf = minreal(C1*Gs/(1 + (C1+C2)*Gs));
Gz = c2d(Gf, 0.02);
Gz.Variable = 'z^-1';

% Gz = z^-1*(0.09142 + 0.2228*z^-1)/(1 - 0.6893*z^-1 - 0.108*z^-2);
% Gs = 0.01595*2/3/s;
% Gz = c2d(Gs, T);

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startcom('COM5', Gz, 0.00);

%  d = 30.0023, 12.6388, eo = -4.4156e-13, a = 71.51,
%  w = 4.6047, Gjw = 1.872 <-2.9639

%% ESTADO INCIAL
[r, rf, rd, rq, y, e, er, u, ur, pwm, f] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%% MISC
umax = 1.0; % maximum pwm
set  = 0.0; % fixed setpoint

%% RELAY
d = 0.01; % u amplitude
off = 0.0; % u mean
eps = 3.0; % histeresis
ref = 0; % initial reference
edge_comb = 4; % comb waits at least 4 edges

d = 1;
eps = 0.5;

%robust: unaffected by noises or disturbances
%resilient: retries itself after stuck for too long
%anti-fragile: updates the mean reference after a try or switch
nr = 0;
up = 1;
edge = 0;
semi = [0; 0];
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
QF = 5.0;  %^quality, ^settling time, ^precision

%for QF > 5.0, phase error > 50/QF° for a band equal the 1st harmonic.
%for QF = 1.0, there is a time delay 

%calculates comb filter coefficient based on filter quality
[kf, qf] = deal(o);
af = 1 / cos(pi / (2 * QF)) + cos(pi / (2 * QF));
af = af + sqrt(af^2 - 4);
af = (af - sqrt(af^2 - 4)) / 2;
a  = af;

%% PID
kp=7.987*10^-4;td=0.3487;ti=0.0;

%calculates controllers' numerator
gd   = @(kp, td) [kp*td/T -kp*2*td/T kp*td/T];
gpi  = @(kp, ti) [kp*(1 + T/(2*ti)) -kp*(1 - T/(2*ti))];
gpid = @(kp, td, ti) [kp*(1 + T/(2*ti) + td/T) -kp*(1 + 2*td/T - T/(2*ti)) kp*td/T];
%gains
ke = flip(gd(kp,td));
ku = 1.0;
o = 205;

%% LOOP DE CONTROLE
for k = o:n
    %LEITURA
    time = tic;
    y(k) = read();
    
    %REFERÊNCIA
%     if kf < (k - semi(~up+1))
%         kf = kf + 1;
%     elseif kf > (k - semi(~up+1))
%         kf = kf - 1;
%     end
    foo(k,:) = semi;
    kf = k - semi(~up+1);
    f(k) = rmax + eps < ymax && rmin - eps > ymin;
%     af = f(k)*a;
    af = a;
    rd(k) = -af  * rd(kf) + (y(k) + y(kf)) * (1  + af) / 2;                            % type 0
    rd(k) = -af  * rd(kf) + 1/4*((3 + af)*y(k) + 2*(1 + af)*y(kf) + (af - 1)*y(k-nr)); % type 1
    rf(k) = (rd(k) + rd(kf))/2;
%     if semi(up+1) > nr && abs(set - rd(k)) > eps
%         semi(~up+1) = semi(~up+1) + 1;
%         semi(up+1) = semi(up+1) - 1;
%         rf(k) = rd(k);
% %     	rd(k) = rd(k-1); % type 0
%     end
    if edge < edge_comb
        rd(k) = set;
    end
    r(k) = rd(k);
    if ~up
        ymax = max(y(k), ymax);
        rmax = max(r(k), rmax);
    else
        ymin = min(y(k), ymin);
        rmin = min(r(k), rmin);
    end
    
%     kf = k - semi(~up+1);
%     if semi(up+1) < nr
%         rd(k) = -af  * rd(kf) + (y(k) + y(kf)) * (1  + af) / 2;                            %type 0
%         rd(k) = -af  * rd(kf) + 1/4*((3 + af)*y(k) + 2*(1 + af)*y(kf) + (af - 1)*y(k-nr)); %type 1
%     elseif (yp && ~up) || (~yp && up)
%         retry = retry + 1;
%         if up
%             ref = ymin;
%         else
%             ref = ymax;
%         end
%         rd(k) = ref;
%         up = ~up;
%     else
%         rd(k) = rd(k-1);
%     end
%     rf(k) = (rd(k) + rd(kf))/2;
%     if ~up
%         ymax = max(y(k), ymax);
%         rmax = max(rd(k), rmax);
%     else
%         ymin = min(y(k), ymin);
%         rmin = min(rd(k), rmin);
%     end
%     %COMB FILTER
%     f(k) = rmax + eps < ymax && rmin - eps > ymin; % not good
%     af = f(k)*abs(af) - ~f(k)*abs(af);
%     r(k) = rd(k);
    
    %ERRO
    e(k) = r(k) - y(k);
    
    %CONTROLE
    %MQNR
%     if k < n/2
%         u(k) = 50 + 5*randn();
%     else
%         u(k) = 80 + 5*randn();
%     end
    if (e(k) >= eps && ~up) || (e(k) <= -eps && up)
      semi(up + 1) = saw;
      nr = semi(~up+1) + semi(up+1);
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
      rf(k) = rd(k);
    else
      saw = saw + 1;
    end
%     if semi(up+1) > nr
%       if abs(e(k)) < eps
%         semi(~up+1) = semi(~up+1) + 1;
%         semi(up+1) = semi(up+1) + 1;
%       end
%       semi(up+1) = semi(up+1) - 1;
%       rf(k) = rd(k);
%     end
%     if (up && (e(k) - e(k-1))/T > 10.0) || (~up && (e(k) - e(k-1))/T < -10.0)
%         d = 0;
%     else
%         d = 0.01;
%     end
    
    %RELAY
    er(k) = set - rd(k);
%     ur(k) = ku*ur(k-1) + ke*er(k-2:k);
%     ur(k) = min(max(ur(k), -umax+d), umax-d); % saturando o sinal de controle, fica não linear
    u(k) = ur(k) + up*d - ~up*d - (k>1037)*d;
%     u(k) = off + up*d - ~up*d;
    %PID
%     u(k) = ku*u(k-1) + ke*e(k-2:k);
    %STEP
%     u(k) = 70;

    %SATURAÇÃO
%     pwm(k) = min(max(u(k), -umax), umax);
    pwm(k) = u(k);
    
    %ESCRITA
    write(pwm(k));
    ping(k) = toc(time);
    
    %DELAY
    if isa(stop, 'function_handle')
        while toc(time) < T
        end
    end
    
    %STOP
%     if k > 100
%         break
%     end
end
stop();
fprintf('Duração: %f seconds\n', toc(t0) - toc(time));
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