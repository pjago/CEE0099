%1. Press Run and select "Change Folder"
%2. Modify CONFIG and repeat

clc
clear
format shortg
addpath 'src'

global y r t e u v k

%% CONFIG

T = 0.1;        %time sampling
n = 101;        %sample size
d = 40;

%define functions to calculate r, e, u, v
R = @(k) 82.1;
E = @(y, r, k) r(k) - y(k);
U = @(u, e, k) d;
V = @(u, k) (u(k) > 0)*(u(k) < 100)*u(k) + (u(k) >= 100)*100;

%% I/O

%define transfer function, keeping ioDelay = 1
z = tf('z', T, 'variable', 'z^-1');
Gz = (0.077508 + 0.17161*z^-1)/(1 - 0.8539*z^-1 - 0.02473*z^-2);
Gz.ioDelay = 1;

%set COM at 19200 baud rate at Device Manager
[read, write, plant] = startcom(T, 'COM6', '0.0.0.0:3001', Gz);

%% CONTROL LOOP

%set initial values to zero state
t = (0:(n-1))*T;
[r, y, e, u, v] = deal(zeros(n, 1)); 
ping = nan(n, 1);
t0 = tic;

%LOOP
for k = 1:n
    %READ
    time = tic;
    y(k) = read();

    %SETPOINT
    r(k) = R(k);
    e(k) = E(y, r, k);

    %CONTROL
    u(k) = U(u, e, k);
    v(k) = V(u, k);
    
    %ACTUATE
    write(v(k));
    ping(k) = toc(time);
    
    %DELAY
    if plant
        while toc(time) < T
        end
    end
end

fprintf('Elapsed time: %f seconds\n', toc(t0) - toc(time));
saverun(plant, ping, t, y, r, e, u, v)
write(0);
