% TODO: DELETE THIS IN FAVOR OF STARTCOM (REFACTOR)
%[stop, read, write] = startty(COM)
%[stop, read, write] = startty(COM, Gz)
function [hold, read, write] = startty(COM, varargin)
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
        s.terminator = '';
        s.RecordDetail = 'verbose';
        s.RecordName = 'comlog.txt';
        if isa(Gz, 'tf')
            s.Timeout = Gz.Ts;
        end
        s.InputBufferSize = 2;
        flushinput(s);
        flushoutput(s);
        fopen(s);
        record(s, 'off');
        try
            read   = @() readino(s);
            write  = @(duty) writeino(s, duty);
            hold   = @(period) holdino(s, period);
            warning('off', 'MATLAB:serial:fread:unsuccessfulRead');
            read();
            warning('on', 'MATLAB:serial:fread:unsuccessfulRead');
            write(0);
        catch
            warning('on', 'MATLAB:serial:fread:unsuccessfulRead');
            FOSC = 4000000; % 4 Mhz Crystal
            T0PS = round(FOSC*Gz.Ts/16384); % 16 Prescale
            s.Timeout = T0PS*16384/FOSC;
            read   = @() readpic(s);
            write  = @(duty) writepic(s, duty);
            hold   = @(period) holdpic(s, period);
            fwrite(s, 't'); % Enable sampling interrupts
            fwrite(s, T0PS); % Configure sampling interval
            write(0); % Will only write on next read
            read(); % Blocks until next read
        end
    catch ME1
        if isa(Gz, 'tf') || isa(Gz, 'zpk') || isa(Gz, 'ss')
            Gz = tf(Gz);
            errors = textscan(ME1.message, '%[^\n]', 1);
            disp([errors{end}{:}]);
            disp('Using transfer function model.')
            read   = @() readsim(Gz); 
            write  = @(duty) writesim(duty);
            hold   = @(period) [];
        else
            disp(ME1)
        end
    end
end