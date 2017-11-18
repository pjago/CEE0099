%write simulink block. returns write success (0 or 1)
function success = writeslk(serial, duty)
    try
        writelino(serial, duty);
        success = 1;
    catch
        success = 0;
    end
end
