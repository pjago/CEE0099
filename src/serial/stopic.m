%stopic(serial_port)
function stopic(s)
    fwrite(s, 's', 'uint8');
    fwrite(s, 0, 'uint8');
end
