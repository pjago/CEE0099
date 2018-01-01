%stopino(serial_port)
function stopino(s)
    fwrite(s, '2', 'uint8');
    fwrite(s, 10, 'uint8');
end