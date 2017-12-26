clear
addpath(genpath('src'))
    
%% CONFIGURACAO

COM = 'COM20';

Gzz = {filt([0 0.0378 0.0785], [1 -0.6491 -0.1966], 0.1);
       filt([0 0.1696 0.2306], [1 -0.6761 -0.06456], 0.2);
       filt([0 0.4159 0.394], [1 -0.6629 -0.01249], 0.3); 
 	   filt([0 0.7334 0.6183], [1 -0.5764 -0.01157], 0.4); 
	   filt([0 1.096 0.9359], [1 -0.4376 -0.02607], 0.5); 
       filt([0 0.02258 -0.005138], [1 -0.5751 -0.3476], 0.03)}; 

%% PROCEDIMENTO

for id = 1:length(Gzz)
    itag = num2str(id);
    for folder = {'relay' 'open_step' 'closed_step' 'open_noise' 'cycling'}
        folder = folder{1};
        while length(dir([folder '/' itag '*'])) < 3         
            %% create folder
            
            if ~exist(folder, 'dir')
                mkdir(folder)
            end
            session = dir(folder);
            session = session([session.isdir]);
            next = session(end).name(end);
            if isnan(next) || session(end).name(1) ~= itag
                next = 'a';
            elseif length(dir([folder '/' session(end).name])) == 16
                next = next + 1;
            elseif length(dir([folder '/' session(end).name])) > 2
                rmdir([folder '/' session(end).name], 's');
            end
            if ~exist([folder '/' itag next], 'dir')
                mkdir([folder '/' itag next])
            end

            %% 2 points of operation: (300 +- 10)  e (230 +- 10)

            eval([folder '(30, COM, Gzz{id})']) % 107.03 +- 1.09
            eval([folder '(40, COM, Gzz{id})']) % 153.47 +- 1.32
            eval([folder '(50, COM, Gzz{id})']) % 200.06 +- 1.37
            eval([folder '(60, COM, Gzz{id})']) % 241.44 +- 2.46
            eval([folder '(70, COM, Gzz{id})']) % 281.56 +- 2.31
            eval([folder '(80, COM, Gzz{id})']) % 314.59 +- 2.30
            eval([folder '(90, COM, Gzz{id})']) % 341.09 +- 2.04

        end
    end
end