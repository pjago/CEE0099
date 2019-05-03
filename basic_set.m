%% read and press ctrl + enter for each block!
return
clear

%% libraries (must be at current folder!)
addpath('jsonlab-1.5')

%% Triangular Pulse Input
nlang = 3; %number of linguistic variables
mixing = 0.5; %from 0 to 1, there are at most two variables (required)
span = 400; %distance beetween edges (input span)

b = linspace(-1.0, 1.0, nlang)';
a = b - 1/(nlang - 1)*(1 + mixing)';
c = b + 1/(nlang - 1)*(1 + mixing)';
a(1) = -inf; c(end) = inf; %edges
s = struct('params', mat2cell([a b c]*span/2, ones([1 nlang])),...
    'type', 'triangularPulse');

%% Triangular Pulse Output
nlang = 5; %number of linguistic variables
mixing = 0.5; %from 0 to 1, there are at most two variables (required)
span = 100; %distance beetween edges (input span)

b = linspace((1 + mixing)/(2*nlang), 1.0 - (1 + mixing)/(2*nlang), nlang)';
a = b - (1 + mixing)/(2*nlang)';
c = b + (1 + mixing)/(2*nlang)';
s = struct('params', mat2cell([a b c]*span, ones([1 nlang])),...
    'type', 'triangularPulse');

%% other distributions @todo

%% Save JSON
json = savejson('', s);
json([1:2 end-1:end]) = [];
file = fopen('.\fuzzy\membership\fan\u.json', 'w');
fprintf(file, json);
fclose(file);
