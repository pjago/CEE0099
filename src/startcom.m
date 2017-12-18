%Attempts to connect with plant or model, in that order
%[stop, read, write] = startcom(COM)
%[stop, read, write] = startcom(COM, Gz)
function [stop, read, write] = startcom(COM, varargin)
    if nargin == 2
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
        read   = @() readlino(s);
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