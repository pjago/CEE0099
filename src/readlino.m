%readlino(s, T) (verify the com port and baud rate)
function y = readlino(s, T)
    flushinput(s);
    flushoutput(s);
    fwrite(s, '7', 'uint8'); 
    fwrite(s, 10, 'uint8');
    resposta = fread(s, 2); 
    y = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))))/(7*T);
end