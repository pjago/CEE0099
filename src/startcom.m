%Attempts to connect with plant, virtual or model, in that order
function [read, write, plant] = startcom(T, COM, url, Gz)
    try
        s = serial(COM,'BaudRate',19200);
        s.terminator = 'LF';    
        out = instrfind(s);
        if strcmp(out.status, 'closed')
            fclose(instrfind);
            fopen(s);
        end
        s.Timeout = T/2;
        read   = @() readlino(s, T);
        write  = @(duty) writelino(s, duty);
        plant = 'dc fan';
        read();
    catch ME1
        errors = textscan(ME1.message, '%[^\n]', 1);
        disp([errors{end}{:} ' Trying virtual plant...']);
        try
            rport  = ['http://' url '/true-rpm'];
            wport  = ['http://' url '/pwm'];
            read   = @() getfield(webread(rport), 'true_rpm');
            write  = @(duty) webwrite(wport, 'value', duty);
            plant = 'web fan';
            read();
        catch ME2
            errors = textscan(ME2.message, '%[^\n]', 1);
            disp([errors{end}{:}]); Gz
            read   = @() readsim(Gz); 
            write  = @(duty) writesim(duty); 
            plant = 0;
        end
    end
end