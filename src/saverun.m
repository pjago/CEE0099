function saverun( plant, ping, t, y, r, e, u, v ) % instead pass a timeseries
    if ~plant
        folder = 'sim';
    else
        folder = plant;
    end
    if sum(ping(1:end-1)' > diff(t))
        disp('In-loop latency is too high! Increase your sampling time.')
    end
    if ~exist(folder, 'dir') 
        mkdir(folder);
    end   
    date = datestr(datetime('now'));
    date(date == '-' | date == ':') = '_';
    path = [folder '/' date];
    fig = plotudo(t, y, r, e, u, v, 0, 0);
    save([path '.mat'], 'plant', 'ping', 't', 'y', 'r', 'e', 'u', 'v')
    saveas(fig, [path '.fig'])
    disp(['Plant: ' folder ' Saved at: ' path])
end