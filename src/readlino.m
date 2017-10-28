%readlino(s, T) (verify if the baud rate is correct)
function y = readlino(s, T)
    flushinput(s);
    flushoutput(s);
    fwrite(s, '7', 'uint8'); 
    fwrite(s, 10, 'uint8');
    resposta = fread(s, 2); %Leitura em bits
    y = double(bitor(bitshift(uint16(resposta(1)),8),uint16(resposta(2))))/(7*T);
end