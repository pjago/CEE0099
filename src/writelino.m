%writelino(serial_port, duty_percent)
function writelino(s, value)
    if value == 10
        value = 11; % BKWD: protocol issue 10 is reserved
    end
    fwrite(s, '5', 'uint8'); 
    fwrite(s, value, 'uint8');
    fwrite(s, 10, 'uint8');
end