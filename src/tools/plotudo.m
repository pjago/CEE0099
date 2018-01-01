%[fh1 fh2] = plotudo(t, y, r, e, u, pwm, ...) 
%plotudo(t, y, r, e, u, pwm, 1)    plota -yD, eD, eI
%plotudo(t, y, r, e, u, pwm, ~, 1) plota fft
function [hf, hfu] = plotudo(t, y, r, e, u, pwm, varargin)

    default = {0, 0};
    custom = [varargin default{(1 + length(varargin)):end}];
    [verbose, fft] = custom{:};
    
    T  = mean(diff(t));
    if verbose
        dt = repmat(diff(t)', 1, size(e, 2));
        yD = diff(y)./dt;
        yD(end + 1, :) = NaN;
        eD = diff(e)./dt;
        eD(end + 1, :) = NaN;
        eI(1, :) = nan([1, size(e, 2)]);
        eI = [eI; cumsum((dt.*(e(1:end-1, :) + e(2:end, :))/2))]; 
    end
    
    for k = 1:size(y, 2)
        hf(k) = figure;
        ax1 = subplot(2, 1, 1);
        hold on
        grid on
        xlabel('Tempo (s)')
        if k == 1
            title('Response')
        else
            title(sprintf('Response %d', k))
        end
        stairs(t', at(r, ':', k), 'r--', 'LineWidth', 0.5)
        stairs(t', y(:, k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
        stairs(t', at(r, ':', k), 'r--', 'LineWidth', 0.5);        
        hold off
        legend('r', 'y', 'Location', 'Best');
        legend('boxoff');
        
        ax2 = subplot(2, 1, 2);
        hold on
        grid on
        xlabel('Tempo (s)')
        if verbose
            stairs(t', at(u,    ':', k), 'k', 'LineWidth', 0.5);
            stairs(t', at(pwm, ':', k), 'ko', 'MarkerSize', 2.5, 'LineWidth', 0.5);
            stairs(t', at(e,    ':', k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
            stairs(t', at(eI,   ':', k), 'LineWidth', 0.5, 'Color', [0.9290 0.6940 0.1250]);
            stairs(t', at(eD,   ':', k), 'LineWidth', 0.5, 'Color', [0.850 0.325 0.098]);
            stairs(t', at(-yD,  ':', k), 'o', 'MarkerSize', 2.5, 'LineWidth', 0.5, 'Color', [0.850 0.325 0.098]);
            plot(t, zeros(size(t)), 'r--', 'LineWidth', 0.5);
            legend('u', 'pwm', 'e', 'eI', 'eD', '-yD');
        else
            stairs(t', at(u,    ':', k), 'k', 'LineWidth', 0.5);
            stairs(t', at(pwm, ':', k), 'ko', 'MarkerSize', 2.5, 'LineWidth', 0.5);
            stairs(t', at(e,    ':', k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
            plot(t, zeros(size(t)), 'r--', 'LineWidth', 0.5);
            legend('u', 'pwm', 'e', 'Location', 'Best');
            legend('boxoff')
        end
        hold off
        linkaxes([ax1 ax2], 'x')
    end
    
    %FOURIER
    if fft
        for k = 1:size(y, 2)
            hfu(k) = figure;
            colormap('default')
            hold on
            grid on
            if verbose
                plot_fft( y(      ':', k), 1/T, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
                plot_fft(at(r,    ':', k), 1/T, 'r--', 'LineWidth', 0.5);
                plot_fft(at(u,    ':', k), 1/T, 'k', 'LineWidth', 0.5);
                plot_fft(at(pwm, ':', k), 1/T, 'ko', 'MarkerSize', 4, 'LineWidth', 0.5);
                plot_fft(at(e,    ':', k), 1/T, 'x', 'MarkerSize', 4, 'Color', [0 0.447 0.741]);
                plot_fft(at(eI,   ':', k), 1/T, 'x', 'MarkerSize', 4, 'Color', [0.9290 0.6940 0.1250]);
                plot_fft(at(eD,   ':', k), 1/T, 'x', 'MarkerSize', 4, 'Color', [0.850 0.325 0.098]);
                plot_fft(at(-yD,  ':', k), 1/T, 'o', 'MarkerSize', 4, 'Color', [0.850 0.325 0.098]);
                legend('y', 'r', 'u', 'pwm', 'e', 'eI', 'eD', '-yD');
            else
                plot_fft( y(      ':', k), 1/T, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
                plot_fft(at(r,    ':', k), 1/T, 'r--', 'LineWidth', 0.5);
                plot_fft(at(u,    ':', k), 1/T, 'k', 'LineWidth', 0.5);
                plot_fft(at(pwm, ':', k), 1/T, 'ko', 'MarkerSize', 4, 'LineWidth', 0.5);
                legend('y', 'r', 'u', 'pwm', 'Location', 'Best');
                legend('boxoff')
            end
            title(sprintf('Fourier %d', k))
            hold off
        end
    end
end

