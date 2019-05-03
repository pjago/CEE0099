%Attempts to connect with plant or model, in that order
%[stop, read, write] = startcom(COM)
%[stop, read, write] = startcom(COM, Gz)
function [stop, read, write, plant] = startcom(COM, varargin)
    if nargin >= 2
        Gz = varargin{1};
    end
    if nargin >= 3
        noise_amp = varargin{2};
    else
        noise_amp = 0;
    end
    try
        s = serial(COM,'BaudRate',19200);
        s.terminator = 'LF';
        s.RecordDetail = 'verbose';
        s.RecordName = 'comlog.txt';
        out = instrfind(s);
        if strcmp(out.status, 'closed')
            fclose(instrfind);
            fopen(s);
        end
        record(s, 'off');
        read   = @() readlino(s);
        write  = @(duty) writelino(s, duty);
        stop   = @() stopino(s);
        plant = true;
        read();
    catch ME1
        errors = textscan(ME1.message, '%[^\n]', 1);
        disp([errors{end}{:}]); Gz
        read   = @() readsim(Gz) + noise_amp*randn(); 
        write  = @(duty) writesim(duty);
        stop   = 0;
        plant = false;
    end
end