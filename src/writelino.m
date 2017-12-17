%writelino(serial_port, duty_percent)
function writelino(s, value)
    flushinput(s);
    flushoutput(s);
    fwrite(s, '5', 'uint8');
    fwrite(s, value, 'uint8'); 
    fwrite(s, 10, 'uint8');
end