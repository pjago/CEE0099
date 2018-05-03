% dmplot, ureal, loopsens, wcsens, tfestimate, tfest, mscohere, sigma
clear
warning('off', 'Control:analysis:MarginUnstable')
warning('off', 'Control:ltiobject:UseSSforInternalDelay')
warning('off', 'symbolic:solve:SolutionsDependOnConditions')
map = @(fsym, x) eval(subs(fsym, x)); 

N = 101; % number of samples
W = linspace(0, 1, N); % normalized frequency
K = 0:(N-1); % time sample vector

%% Represent plant by ZeroPoleKain

O = eps;
P = [eps 0.5 0.5]';
I = @(x) eye(length(x));
Kp = 2;
Gp = zpk(O, P, Kp*prod(1 - P)/prod(1 - O), 1, 'Variable', 'z^-1');

%% First analysis

[~, piWs] = getPeakGain(1/(1 + Gp));
Gjws = evalfr(Gp, exp(1j*piWs));
Wu = 1;
Ws = piWs/pi;
Ku = margin(Gp);
Ks = -1/Gjws;

%% Closing the loop

Kc = 1;
L  = Gp*Kc;
Gf = minreal(L/(1+L));
Pf = [Gf.P{:}];

%% Robustness

Si = minreal(1/(1+L));
Ti = minreal(L/(1+L));

%% Plotting nyquist and rlocus

hf1 = figure(1);
hf1.Position = [1 248 1024 428];

subplot(1,2,1)
hold on
nyquist(Gp)
axis equal
grid on

subplot(1,2,2)
hold on
rlocus(Gp)
plot(real(Pf), imag(Pf), 'rx')
axis equal
grid on

%% Real mother f'ing syms

syms r w kp kc
assume(r,  'real')
assume(w,  'real')
assume(kp, 'real')
o = sym('o%d', [length(O) 1]);
p = sym('p%d', [length(P) 1]);
z = r*exp(1i*pi*w);

Gp = zpk(O, P, Kp*prod(1 - P)/prod(1 - O), 1, 'Variable', 'z^-1');
gp = kp*det(I(O) - diag(o)*z^-1)/det(I(O) - diag(o)) / ...
     (  det(I(P) - diag(p)*z^-1)/det(I(P) - diag(p)) );
l  = gp*kc;
gf = l/(1 + l);
si = 1/(1 + l);
ti = l/(1 + l);

ku = solve(subs(1/si, [r w], [1 -1]), kc);
ks = solve(subs(diff(abs(1/si), r)==1i*pi/r*diff(abs(1/si), w), r, 1), kc);

% Input sensitivity function (and complement)
sijw = subs(si, [r; o; p; kp; kc], [1; O; P; Kp; Kc]);
tijw = subs(ti, [r; o; p; kp; kc], [1; O; P; Kp; Kc]);

Sijw = map(sijw, W);
Tijw = map(tijw, W);

% % Critical compensator
% kcjw = subs(-1/gp, [r; o; p; kp], [1; O; P; Kp]);

%% Plotting characteristic equations

hf2 = figure(2);
hold on

subplot(2, 1, 1)
hold on
fplot(abs(tijw), [eps 1]);
fplot(abs(sijw), [eps 1]);
plot(Ws, map(abs(sijw), Ws), 'rx')
plot(Ws, map(abs(tijw), Ws), 'rx')
ax1 = gca;
set(ax1, 'Xtick', [])
set(ax1, 'Position',[0.13 0.54 0.775 0.39]);
title('Bode Diagram')
ylabel('Magnitude (unit)')

subplot(2, 1, 2)
hold on
fplot(angle(tijw), [eps 1]);
fplot(angle(sijw), [eps 1]);
ax2 = gca;
set(ax2, 'Position',[0.13 0.11 0.775 0.39]);
xlabel('Frequency (xπ rad/sample)')
ylabel('Phase (rad)')
linkaxes([ax1 ax2], 'x')