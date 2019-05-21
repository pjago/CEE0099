%holdino(serial_port, period)
function holdino(s, period)
    if period == Inf
        fwrite(s, '2', 'uint8');
        fwrite(s, 10, 'uint8');
    elseif period > 0
        java.lang.Thread.sleep(1000*period);
    end
end