%% COMNECT
COM = '/dev/ttyUSB0';
out = instrfind('Port', COM);
if ~isempty(out)
    fclose(out);
    delete(out);
end
s = serial(COM, 'BaudRate', 19200);
fopen(s);
%% RUN
for j = 1:10000
    fwrite(s, '5')
    fwrite(s, 100)
    fwrite(s, 10)
    fwrite(s, '7')
    fwrite(s, 10)
    echo = fread(s, 2)
end