%holdpic(serial_port, order, period)
function holdpic(s, period)
    if period == Inf
        fwrite(s, 's', 'uint8');
        fwrite(s, 0, 'uint8');
    elseif period > 0
        java.lang.Thread.sleep(1000*period);
    end
end