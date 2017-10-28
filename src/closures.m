function uses = closures(fh)
    uses = [];
    for k = 1:length(fh)
        fstr = toString(fh{k});
        vrex = '([a-zA-Z_][a-zA-Z0-9]*)*';
        arex = ['^@\((' vrex  '|,|~)+\)'];
        vars = unique(regexp(fstr, vrex, 'match'));
        args = regexp(regexp(fstr, arex, 'match', 'once'), vrex, 'match');
        uses = [uses setdiff(vars, args)];
    end
    uses = unique(uses);
end