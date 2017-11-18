%read simulink block. (verify the com port and baud rate)
function [y, success] = readslk(serial, T)
    try
        y = readlino(serial, T);
        success = 1;
    catch
        success = 0;
    end
end