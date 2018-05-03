% TODO: garantir a oscilação, principalmente quando o modelo não for exato
% TODO: check relation between notch quality and transient

function [Gjw, spin, cycle] = relay (Gz, COM, set, varargin)
% relay
addpath(genpath('src'))

global t y r e u pwm ping k

%% VARARGIN

if nargin >= 4
    D = varargin{1};
else
    D = 0.5;
end

if nargin >= 5
    ws = varargin{2};
    ns = 2*round(1/ws); % input point
    ws = 2*pi/(ns*Gz.Ts);
else
    % [~, ~, ws] = margin(Gz); % crossover point
    % ns = 2*round(pi/(ws*Gz.Ts));
    % ws = 2*pi/(ns*Gz.Ts);
    [~, ws] = getPeakGain(1/(1 + Gz)); % sensible point
    ns = 2*round(pi/(ws*Gz.Ts));
    ws = 2*pi/(ns*Gz.Ts);
end

%% CONFIGURACAO

Gjw = evalfr(Gz, exp(1j*ws*Gz.Ts));
kp = dcgain(Gz);
ref = kp*set;

% Adicione o nome de variaveis que queira salvar
toSave = {'ping', 'Gjw', 'ws', 'eps', 'eo', 'd', 'a', 't', 'y', 'r', 'e', 'u', 'pwm', 'spin', 'cycle'};

D = round(D*ns)/ns;
d = round((0.10*100*kp)/(abs(Gjw)*4/pi)); % ~amplitude
a = d*abs(Gjw)*4/pi;
eps = -d*imag(Gjw)*4/pi; % ~frequency (TODO: PRECISE FORMULA)

uo = (D-0.5)*2*d;
ko = 2*d/(pi*a*sqrt(1 - (eps/a)^2));
eo = uo/ko; % ~duty-cycle (dc correction)
yo = kp*uo;

% % good replacement for half-cycles
% eps = eps - abs(eo);
% eo = 0;

ro = eo + yo;
if abs(ro) < kp*d - eps
    disp('OK!')
end

%% I/O

%ajuste a COM e o baud rate de 19200, em Gerenciador de Dispositivos
[stop, read, write] = startty(COM, Gz);

%% TUNING

info = stepinfo(Gz);
duration = (info.SettlingTime + 20*ns*Gz.Ts);

T = Gz.Ts;                  %tempo de amostragem
n = round(duration/T) + 1;  %numero de amostras
t = (0:(n-1))*T;            %vetor de tempo

Qf = 1.5; % TODO: (settling time) (overshoot) (uncertainty) (increase with time)
BW = (2/ns)/Qf;

% % 1st notch filter (DOESN'T WORK WITH NS = 2) (FEWER FREQUENCY RANGE)
% [bn, an] = iirnotch(2/ns, BW);

% odd comb filter (WORKS WITH NS = 2) (MAX+ FREQUENCY RANGE)
ai = sec(pi/(2*Qf)) + cos(pi/(2*Qf));
am = ai + sqrt(ai^2 - 4);
af = (am - sqrt(am^2 - 4))/2;

% % natural comb filter (HAS ERROR WHEN SWITCHED) (POSSIBLY ADAPTS DUTY)
R  = af^(2/ns); % close enough

%% ESTADO INCIAL

[r, y, e, u, pwm, saw, env, er, ef] = deal(zeros(n, 1));
ping = nan(n, 1);
t0 = tic;

%limit cycles
cycle = false(n, 1);
spin = zeros(n, 1);

% for k = 1:n
%     time = tic;
%     y(k) = read();
%     r(k) = y(k);
%     e(k) = 0;
%     er(k) = 0;
%     u(k) = set + uo;
%     pwm(k) = u(k);
%     write(pwm(k));
%     ping(k) = toc(time);
%     stop(T - ping(k));
% end
% 
% r(1:ns+1) = r(end-ns:end);
% y(1:ns+1) = y(end-ns:end);
% e(1:ns+1) = e(end-ns:end);
% u(1:ns+1) = u(end-ns:end);
% pwm(1:ns+1) = pwm(end-ns:end);

%% LOOP DE CONTROLE

for k = (ns+2):n
    %LEITURA
    time = tic;
    y(k) = read();
    
    %REFERENCIA E ERRO
%     r(k) = ref + yo; % has the problem where it can't correct the DC
%     r(k) = (-an(2:end)*r(k-1:-1:k-2) + bn*(y(k:-1:k-2))); % has the
%     problem where for slow frequencies the switching cannot be reached
%     r(k) = (r(k-ns/2)*(-af) + (y(k) + y(k-ns/2))*(1+af)/2); % has
%     the problem where it cannot capture asymmetric switchings
    r(k) = (r(k-1) + R^ns*(r(k-ns) - r(k-ns-1)) + ((y(k) - y(k-ns)) + ...
    R*(y(k-ns-1) - y(k-1)))/(ns*(1 - R)/(1 - R^ns))); %has the problem
%     where it introduces steady state error when switched on/off
%     r(k) = cycle(k-1) * r(k) + ~cycle(k-1) * (ref + yo); % entrance point
    e(k) = eo + r(k) - y(k);
    er(k) = r(k) - (ref + yo);

    %CONTROLE
    env(k) = (D - 0.5 - 0.5*spin(k-1))*ns;
    saw(k) = -spin(k-1);
    if e(k) >= eps && spin(k-1) == -1
       cycle(k) = ns*(D - 1) == saw(k-1) && ns*D == saw(k+ns*(D-1)-1);
       spin(k) = 1;
    elseif e(k) <= -eps && spin(k-1) == 1
       cycle(k) = ns*D == saw(k-1) && ns*(D - 1) == saw(k-ns*D-1);
       spin(k) = -1;
    elseif saw(k-1) == env(k-1) % fifa (forced if almost)
       cycle(k) = false;
       spin(k) = -spin(k-1) + ~spin(k-1)*sign(ref + yo);
       saw(k) = spin(k);
    else
       cycle(k) = cycle(k-1);
       spin(k) = spin(k-1);
       env(k) = (D - 0.5 + 0.5*spin(k-1))*ns;
       saw(k) = saw(k-1) + spin(k-1);
    end
    u(k) = set + d*spin(k);
    ef(k) = (1 - 2/ns*abs(env(k)));
    
    %SATURACAO
    if u(k) > 100
        pwm(k) = 100;
    elseif u(k) < 0
        pwm(k) = 0;
    else
        pwm(k) = round(u(k));
    end

    %ESCRITA
    write(pwm(k));
    ping(k) = toc(time);
    stop(T - ping(k));
end
stop(Inf);
fprintf('Duracao: %f seconds\n', toc(t0) - toc(time));
if sum(ping(1:end-1)' > T)
    disp('In-loop latency is too high! Increase your sampling time.')
end

%% ANALYSIS

try %cleaning this once
    disp([10 '-> d = ' num2str(d) ', eps = ' num2str(eps) ', eo = ' num2str(eo) ', a = ' num2str(a) ', w = ' num2str(ws) ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw))])
%     settled = cycle;
    settled = ((0:n-1)*T > info.SettlingTime + 2*ns*T)';
    edg = settled.*[0; diff(spin)]/2;
    bet = logical(edg) | logical(circshift(edg, -1));
    d = mean((pwm(logical(edg)) - mean(pwm)).*edg(logical(edg)));
    epsaux = e(bet).*(edg(bet) + circshift(edg(bet), -1));
    if ~isempty(epsaux) % todo: if find 1 cycle
        eo = mean(e(settled));
        eps = mean(epsaux);
    end
    % for measuring a, find the corresponding peak in beetween two edges!
    edg_id = sign(edg).*sign(spin.*settled).*(1:n)';
    edg_id(edg_id == 0) = [];
    aaux = [];
    for ide = 1:length(edg_id)-1
        fst = edg_id(ide);
        lst = edg_id(ide+1)-1;
        aaux(ide, 1) = max(sign(edg(fst))*e(fst:lst));
    end
    if ~isempty(aaux) % todo: if find 1 cycle
        a = median(aaux);
    else
        a = peak2peak(e(round(end/2):end))/2;
    end
    ws = pi*maxfreq(u)/T;
%     Gjw = -pi/(4*d)*sqrt(a^2 - eps^2) - 1j*pi/(4*d)*eps;
    Gjw = -1/(2*d/(pi*a)*(sqrt(1 - ((eps + eo)/a)^2) + sqrt(1 - ((eps - eo)/a)^2)) - 1j*4*d*eps/(pi*a^2));
    disp(['<- d = ' num2str(d) ', eps = ' num2str(eps) ', eo = ' num2str(eo) ', a = ' num2str(a) ', w = ' num2str(ws) ', Gjw = ' num2str(abs(Gjw)) ' <' num2str(angle(Gjw)) 10])
catch ME % todo: find the Black Swan
    eps = NaN;
    a = NaN;
    disp(['Broke! ' ME.message])
end

%% PLOT & SAVE

fig = plotudo(t, y, r, e, u, pwm);
figure(fig)
subplot(2, 1, 2)
refline(0, eps)
refline(0, -eps)
hr1 = refline(0, ro - kp*d);
hr2 = refline(0, ro + kp*d);
hr1.Color = 'm';
hr2.Color = 'm';
legend('off')

% figure
% subplot(2, 1, 1)
% hold on
% stairs(t, er)
% stairs(t, ef)
% legend('er', 'normalized ef', 'Location', 'best')
% subplot(2, 1, 2)
% hold on
% stairs(t, saw)
% stairs(t, env)
% stairs(t, cycle, 'k')
% legend('saw', 'env', 'cycle', 'Location', 'best')

folder = 'relay';
if ~exist(folder, 'dir')
    mkdir(folder);
end
session = dir(folder);
session = session([session.isdir]);
if length(session) > 2
    folder = [folder '/' session(end).name];
end
date = datestr(datetime('now'));
date(date == '-' | date == ':') = '_';
path = [folder '/' date];
save([path '.mat'], toSave{:})
saveas(fig, [path '.fig'])
disp(['Plant: ' folder ' Saved at: ' path 10])
close(fig)
