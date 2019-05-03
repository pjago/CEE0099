%[fh1 fh2] = plotudo(t, y, r, e, u, pwm, ...)
%plotudo(t, y, r, e, u, pwm, 1)    plota -yD, eD, eI
%plotudo(t, y, r, e, u, pwm, ~, 1) plota fft
%plotudo(t, y, r, e, u, pwm, ~, ~, 1) plota yyaxis
%plotudo(t, y, r, e, u, pwm, ~, ~, ~, t0) tempo zero ? em t0
function [hf, hfu] = plotudo(t, y, r, e, u, pwm, varargin)
    opt = {0.0};
    subplot = @(m,n,p) subtightplot(m,n,p,opt{:}); 
    
    hf = [];
    hfu = [];
    
    T = mean(diff(t));
    ko = round(t(1)/T) + 1;
    kf = round(t(end)/T) + 1;
    y = y(ko:kf, :);
    r = r(ko:kf, :);
    e = e(ko:kf, :);
    u = u(ko:kf, :);
    pwm = pwm(ko:kf, :);
    t = t - t(1);
    
    default = {0, 0, 1, 1};
    custom = [varargin default{(1 + length(varargin)):end}];
    [verbose, fft, yy, start] = custom{:};
    t = t - t(start);
    
    if verbose
        dt = repmat(diff(t)', 1, size(e, 2));
        yD = diff(y)./dt;
        yD(end + 1, :) = NaN;
        eD = diff(e)./dt;
        eD(end + 1, :) = NaN;
        eI(1, :) = nan([1, size(e, 2)]);
        eI = [eI; cumsum((dt.*(e(1:end-1, :) + e(2:end, :))/2))]; 
    end
    
    if ~yy
        for k = 1:size(y, 2)
            hf(k) = figure;
            ax1 = subplot(2, 1, 1);
            hold on
            if k == 1
                title('')
            else
                title(sprintf('%d', k))
            end
            stairs(t', at(r, ':', k), 'r--', 'LineWidth', 0.5)
            stairs(t', y(:, k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
            stairs(t', at(r, ':', k), 'r--', 'LineWidth', 0.5);        
            hold off
            legend('r', 'y', 'Location', 'SouthEast');
            set(gca, 'XTick',[], 'YAxisLocation','right')
            legend('boxoff');

            ax2 = subplot(2, 1, 2);
            hold on
%             xlabel('Tempo (s)')
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
%                 stairs(t', at(e, ':', k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
                stairs(t', at(u,    ':', k), 'k', 'LineWidth', 0.5);
%                 [~,hLine1,hLine2] = plotyy(t', at(e, ':', k), t', at(u,    ':', k));
%                 hLine1.LineWidth = 1.5;
%                 hLine1.Color = [0 0.447 0.741];
%                 hLine2.LineWidth = 0.5;
%                 hLine2.Color = 'k';
%                 stairs(t', at(pwm, ':', k), 'ko', 'MarkerSize', 2.5, 'LineWidth', 0.5);
                %HARD CODE CORE CODE CORE CODE
%                 plot(t, zeros(size(t)), 'r--', 'LineWidth', 0.5);
%                 legend('u', 'pwm', 'Location', 'Best');
                legend('u', 'Location', 'SouthEast');
%                 legend('e', 'u', 'Location', 'SouthEast');
                legend('boxoff')
            end
            hold off
            linkaxes([ax1 ax2], 'x')
            subplot(2, 1, 1)
        end
    else
        %BUG: só normaliza com degrais
        %r = r*median(y)/median(r);

        for k = 1:size(y, 2)
            hf(k) = figure;
            set(hf(k),'defaultAxesColorOrder',[0.6*[0 0.447 0.741]; [0 0 0]]);
            
            yyaxis left
            hold on
            %grid on
            xlabel('Tempo (s)')
            if k == 1
                title('')
            else
                title(sprintf('%d', k))
            end
            stairs(t', at(r, ':', k), 'r--', 'LineWidth', 0.5);
            stairs(t', y(:, k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
%             hold off
%             legend('r', 'y', 'Location', 'Best');
%             legend('boxoff');

            yyaxis right
            hold on
            %grid on
            xlabel('Tempo (s)')
            if verbose
                stairs(t', at(u,    ':', k), 'k', 'LineWidth', 0.5);
                %stairs(t', at(pwm, ':', k), 'ko', 'MarkerSize', 2.5, 'LineWidth', 0.5);
                stairs(t', at(e,    ':', k), 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
                stairs(t', at(eI,   ':', k), 'LineWidth', 0.5, 'Color', [0.9290 0.6940 0.1250]);
                stairs(t', at(eD,   ':', k), 'LineWidth', 0.5, 'Color', [0.850 0.325 0.098]);
                stairs(t', at(-yD,  ':', k), 'o', 'MarkerSize', 2.5, 'LineWidth', 0.5, 'Color', [0.850 0.325 0.098]);
                plot(t, zeros(size(t)), 'r--', 'LineWidth', 0.5);
                legend('u', 'e', 'eI', 'eD', '-yD');
            else
                stairs(t', at(u,    ':', k), 'k', 'LineWidth', 0.5);
                stairs(t', at(pwm, ':', k), 'ko', 'MarkerSize', 2.5, 'LineWidth', 0.5);
            end
            legend('r', 'y', 'u', 'pwm', 'Location', 'Best');
            legend('boxoff')
            hold off
        end
    end
    xlim([t(1) t(end)])
    
    %FOURIER
    if fft
        for k = 1:size(y, 2)
            hfu(k) = figure;
            colormap('default')
            hold on
            %grid on
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
            if k == 1
                title('')
            else
                title(sprintf('%d', k))
            end
            hold off
        end
    end
end