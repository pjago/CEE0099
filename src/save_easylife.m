function save_easylife( folder, ping, t, y, r, e, u, v ) % instead pass a timeseries
    if ~exist(folder, 'dir') 
        mkdir(folder);
    end   
    path = [folder '/' datestr(datetime('now'))];
    fig = plotudo(t, y, r, e, u, v, 0, 0);
    save([path '.mat'], 'ping', 't', 'y', 'r', 'e', 'u', 'v')
    saveas(fig, [path '.fig'])
    disp(['Saved at: ' path])
end