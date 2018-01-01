%readpic(serial_port)
function y = readpic(s)
    y = fread(s, 1, 'uint16');
end