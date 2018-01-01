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
fwrite(s, 't')
fwrite(s, 0)
for j = 1:10000
    fwrite(s, 'x')
    fwrite(s, 100)
    echo = fread(s, 2)
end
fwrite(s, 's')
fwrite(s, 0)
