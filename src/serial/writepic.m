%writepic(serial_port, duty_percent)
function writepic(s, value)
    fwrite(s, 'x', 'uint8'); 
    fwrite(s, value, 'uint8');
end