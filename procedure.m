function procedure (COM)

addpath(genpath('src'))
    
%% CONFIGURACAO

% VERSION INO
% Gz = {filt([0 0 0.03847], [1 -0.686 -0.2374], 0.05);
%       filt([0 0.04 0.1121], [1 -0.6167 -0.2307], 0.10);
%       filt([0 0.12 0.1548], [1 -0.7922 -0.02584], 0.15);
%       filt([0 0.2 0.2556], [1 -0.8092 0.03322], 0.20);
%       filt([0 0.36 0.2959], [1 -0.8061 0.06284], 0.25);};

% VERSION PIC
Gz = {filt([0 0 0 0.05081], [1 -0.4592 -0.4462], 0.05);
      filt([0 0 0.06 0.08503], [1 -0.7322 -0.1252], 0.10);
      filt([0 0 0.12 0.2008], [1 -0.6905 -0.1025], 0.15);
      filt([0 0 0.22 0.2486], [1 -0.8084 0.0371], 0.20);
      filt([0 0 0.34 0.3804], [1 -0.7344 0.01274], 0.25);};

folder = 'relay';

% FIND OUT WHICH FREQUENCIES HAVE SULC
% TODO: FIND OUT WHICH VALUES OF EPS MAY MAP TO TWO OR MORE SULCS
% TODO: FIND OUT A NICE WAY OF REFERENCING PAPERS IN CODE
Mtil = 15; % todo: instead stop until it no longer SULCS
SULC = nan(length(Gz), Mtil);
for gg = 1:length(Gz)
    Gs{gg, 1} = ss(Gz{gg});
    Gss = Gs{gg};
    I = eye(size(Gss.A));
    for M = 1:Mtil % half cycles
        Gjw = evalfr(Gss, exp(1j*1/M*pi));
        eps = -imag(Gjw)*4/pi; % ~frequency (TODO: PRECISE FORMULA)
        L = 1;
        xstar = (Gss.A^M + I)\(Gss.A^M - I)/(Gss.A - I)*Gss.B;
        if Gss.C*xstar < eps
            L = 0;
        end
        for i = 1:M-1
            xi = Gss.A^i*xstar - (Gss.A - I)\(Gss.A^i - I)*Gss.B;
            if Gss.C*xi <= -eps
                L = 0;
            end
        end
        SULC(gg, M) = L;
    end
end
SULCsum = sum(SULC, 2);

setpoints = 1./(1:Mtil); % choose based on Gz
params = [];
for set = setpoints
   params = [params; '''' COM '''' ',50,0.5,' num2str(set, '%.5f')];
end

%% PROCEDIMENTO

for gg = 1:length(Gz)
    ggtag = num2str(gg);
    while length(dir([folder '/' ggtag '*'])) < 3         
        %% create folder            
        if ~exist(folder, 'dir')
            mkdir(folder)
        end
        session = dir(folder);
        session = session([session.isdir]);
        next = session(end).name(end);
        if isnan(next) || session(end).name(1) ~= ggtag
            next = 'a';
        elseif length(dir([folder '/' session(end).name])) == 2*SULCsum(gg)+2
            next = next + 1;
        elseif length(dir([folder '/' session(end).name])) > 2
            rmdir([folder '/' session(end).name], 's');
        end
        if ~exist([folder '/' ggtag next], 'dir')
            mkdir([folder '/' ggtag next])
        end
        result = nan(size(params, 1), 1);
        for pp = 1:size(params, 1)
           if SULC(gg, pp)
               result(pp) = eval([folder '(Gz{gg}, ' params(pp,:), ')']);
           else
               result(pp) = NaN;
           end
        end
        figure
        hold on
        nyquist(Gz{gg})
        plot(result, 'r')
    end
end
