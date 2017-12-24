clear
addpath(genpath('src'))
    
%% CONFIGURACAO

COM = '/dev/ttyUSB2';

T = [0.1 0.2 0.3 0.4 0.5 0.03];                       %tempo de amostragem
z1 = tf('z', T(1), 'variable', 'z^-1'); %todo: write in terms of den, num
z2 = tf('z', T(2), 'variable', 'z^-1');
z3 = tf('z', T(3), 'variable', 'z^-1');
z4 = tf('z', T(4), 'variable', 'z^-1');
z5 = tf('z', T(5), 'variable', 'z^-1');
z0 = tf('z', T(6), 'variable', 'z^-1');
Gz = {z1^-1*(0.0378 + 0.0785*z1^-1)/(1 - 0.6491*z1^-1 - 0.1966*z1^-2); % T = 0.1;
      z2^-1*(0.1696 + 0.2306*z2^-1)/(1 - 0.6761*z2^-1 - 0.06456*z2^-2); % T = 0.2;
      z3^-1*(0.4159 + 0.394*z3^-1)/(1 - 0.6629*z3^-1 - 0.01249*z3^-2); % T = 0.3;
	  z4^-1*(0.7334 + 0.6183*z4^-1)/(1 - 0.5764*z4^-1 - 0.01157*z4^-2); % T = 0.4;
	  z5^-1*(1.096 + 0.9359*z5^-1)/(1 - 0.4376*z5^-1 - 0.02607*z5^-2); % T = 0.5;
      z0^-1*(0.02258 - 0.005138*z0^-1)/(1 - 0.5751*z0^-1 - 0.3476*z0^-2)}; % T = 0.03;

%% PROCEDIMENTO

for idx = 1:length(T)
    itag = num2str(idx);
    for folder = {'open_step' 'closed_step' 'relay'}
        while length(dir([folder{1} '/' itag '*'])) < 3
            neurose(itag, folder{1}, COM, Gz{idx})
        end
    end
end