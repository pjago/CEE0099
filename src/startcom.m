%Attempts to connect with plant or model, in that order
%[stop, read, write] = startcom(COM)
%[stop, read, write] = startcom(COM, Gz)
function [stop, read, write] = startcom(COM, varargin)
    if nargin == 2
        Gz = varargin{1};
    else 
        Gz = 0;
    end
    try
        out = instrfind('Port', COM);
        if ~isempty(out)
            fclose(out);
            delete(out);
        end
        s = serial(COM,'BaudRate',19200);
        s.terminator = 'LF';
        s.RecordDetail = 'verbose';
        s.RecordName = 'comlog.txt';
        if isa(Gz, 'tf')
            s.Timeout = Gz.Ts;
        end
        fopen(s);
        record(s, 'off');
        read   = @() readlino(s);
        write  = @(duty) writelino(s, duty);
        stop   = @() stopino(s);
        write(0);
        read();
    catch ME1
        if isa(Gz, 'tf')
            errors = textscan(ME1.message, '%[^\n]', 1);
            disp([errors{end}{:} 10]);
            disp(['Using transfer function model.' 10])
            read   = @() readsim(Gz); 
            write  = @(duty) writesim(duty);
            stop   = 0;
        else
            disp(ME1)
        end
    end
end