%readlino(serial_port)
function y = readlino(s)
    flushinput(s);
    flushoutput(s);
    fwrite(s, '7', 'uint8'); 
    fwrite(s, 10, 'uint8');
    resposta = fread(s, 2); 
    y = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))));
end