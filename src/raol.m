function Gs = raol( Gz )
%RAOL Gs = raol( Gz )
%   Indirect method for estimating a continuous tf
%   Gz must be strictly proper
    [F, G, H] = tf2ss(Gz.num{1}, Gz.den{1});
    d = totaldelay(Gz);
    u = norm(F)/20;
    I = eye(size(F));
    L = F - I;
    P = I;
    M = I;
    i = 1;
    do = true;
    while do
        M = -L*M*u;
        P = P + M;
        i = i + 1;
        do = (norm(M) > 0.01*u) && (i < 1000);
    end
    A = (1/Gz.Ts)*H*P*L;
    B = (1/Gz.Ts)*H*P*G;
    Gs = tf(B, A)*exp(-tf('s')*d*Gz.Ts);
end