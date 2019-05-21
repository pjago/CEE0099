%writelino(serial_port, duty_percent)
function writeino(s, value)
    if value == 10
        value = 11; % BKWD: 10 is reserved as LF
    end
    fwrite(s, '5', 'uint8'); 
    fwrite(s, value, 'uint8');
    fwrite(s, 10, 'uint8');
end