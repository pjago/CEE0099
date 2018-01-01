%readlino(serial_port)
function y = readino(s)
    flushinput(s);
    fwrite(s, '7', 'uint8');
    fwrite(s, 10, 'uint8');
    % BKWD: response is Big Endian, but x86-64 processors are Little Endian 
    resposta = fread(s, 2); 
    y = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))));
end