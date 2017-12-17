%Attempts to connect with plant or model, in that order
function [stop, read, write] = startcom(T, COM, varargin)
    if nargin >= 3
        Gz = varargin{1};
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
        s.Timeout = T/2;
        read   = @() readlino(s, T);
        write  = @(duty) writelino(s, duty);
        stop   = @() stopino(s);
        read();
    catch ME1
        errors = textscan(ME1.message, '%[^\n]', 1);
        disp([errors{end}{:}]); Gz
        read   = @() readsim(Gz); 
        write  = @(duty) writesim(duty);
        stop   = 0;
    end
end