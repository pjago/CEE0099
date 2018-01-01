function procedure ()

addpath(genpath('src'))
    
%% CONFIGURACAO

COM = '/dev/ttyUSB0';

Gz = {filt([0 0.0378 0.0785], [1 -0.6491 -0.1966], 0.1);
      filt([0 0.1696 0.2306], [1 -0.6761 -0.06456], 0.2);
      filt([0 0.4159 0.394], [1 -0.6629 -0.01249], 0.3); 
 	  filt([0 0.7334 0.6183], [1 -0.5764 -0.01157], 0.4); 
	  filt([0 1.096 0.9359], [1 -0.4376 -0.02607], 0.5); 
      filt([0 0.009395 0.02612], [1 -0.5889 -0.3185], 0.05)
      filt([0 0.1 0.05], [1 -1.1 0.2125], 1/7)};

%% PROCEDIMENTO

for id = 1:length(Gz)
    itag = num2str(id);
    for folder = {'open_sine' 'relay' 'open_square' 'open_noise' 'open_step'}
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

            try
                eval([folder '(30, COM, Gz{id})']) 
                eval([folder '(40, COM, Gz{id})']) 
                eval([folder '(50, COM, Gz{id})']) 
                eval([folder '(60, COM, Gz{id})']) 
                eval([folder '(70, COM, Gz{id})']) 
                eval([folder '(80, COM, Gz{id})']) 
                eval([folder '(90, COM, Gz{id})'])
            catch ME
            end

        end
    end
end
