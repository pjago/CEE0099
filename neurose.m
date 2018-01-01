function neurose (itag, folder)
session = dir(folder);
session = session([session.isdir]);

%% Sythesis

% iterate the steady value twice
steady = nan(length(session)-2, 7);
input = nan(length(session)-2, 7);
noise = nan(length(session)-2, 7);
for i = 1:2
    % in each session the work was to increment the input
    for s = 3:length(session) % start from 3 to skip /. and /..
        work = dir([folder '/' session(s).name '/*.mat']);
        if (~isnan(str2double(itag)) && (session(s).name(1) <= '5')) ...
           || (session(s).name(1) <= itag(1))
        % in each work the steady value should be reached midway through
            for w = 1:length(work)
                load([folder '/' session(s).name '/' work(w).name])
                tag(s-2, w) = nanmean(diff(t));  % todo: find a better tag
                % steady
                if i ~= 1 && ~isnan(tset5(s-2, w))
                    half(s-2, w) = floor(tset5(s-2, w)/nanmean(diff(t))) + 1;
                else
                    half(s-2, w) = floor(length(t)/2);
                end
                input(s-2, w) = nanmean(pwm(half(w):end));
                steady(s-2, w) = nanmean(y(half(w):end));
                noise(s-2, w) = std(y(half(w):end));
                % settling
                flipy = flipud(y)/steady(s-2, w);
                downset = find(flipy < 0.95, 1);
                upset = find(flipy > 1.05, 1);
                if isempty(downset)
                    tset5(s-2, w) = NaN;
                elseif isempty(upset)
                    tset5(s-2, w) = t(length(t) - downset + 1);
                else
                    tset5(s-2, w) = t(length(t) - min(downset, upset) + 1);
                end
                % rise
                try
                    gorise = find(y - 0.1*steady(s-2, w) > 0, 1);
                    endrise = find(y - 0.9*steady(s-2, w) > 0, 1);
                    trise(s-2, w) = t(endrise - gorise + 1);
                catch ME
                    trise(s-2, w) = NaN;
                end
                % peak (2nd order)
                [vpeak, lpeak] = findpeaks(y, 'NPeaks', 1);
                if ~isempty(lpeak)
                    tpeak(s-2, w) = t(lpeak);
                    peak(s-2, w) = vpeak;
                else
                    tpeak(s-2, w) = Inf;
                    peak(s-2, w) = NaN;
                end
                % overhoot
                risemax = peak(s-2, w);
                if risemax > steady(s-2, w)
                    overshoot(s-2, w) = (risemax/steady(s-2, w) - 1)*100;
                    % undershoot
                    risemin = min(y(lpeak:2*lpeak));
                    if risemin < steady(s-2, w)
                        undershoot(s-2, w) = (risemin/steady(s-2, w) - 1)*100;
                    else
                        undershoot(s-2, w) = 0;
                    end
                else
                    overshoot(s-2, w) = 0;
                    undershoot(s-2, w) = 0;
                end
            end
        end
    end
end

%% Analysis

if itag == 'T'
    steady = steady./tag;
    noise = noise./tag;
    number = 1;
elseif itag == 'rps'
    steady = steady./(7*tag);
    noise = noise./(7*tag);
    number = 1;
else
    number = str2double(itag); % of different plants
end
group = kmeans(nanmean(tag, 2), number);
[~, sid] = sort(group);
stag = histcounts(group + 0.5, 1:number+1);
sinput = input(sid, :);
ssteady = steady(sid, :);
snoise = noise(sid, :);
applied = [];
expected = [];
margin = [];
for i = 1:length(stag) % todo: find easier way of doing this
    sidtag = sum(stag(1:i-1)) + (1:stag(i));
    taginput = sinput(sidtag, :);
    tagsteady = ssteady(sidtag, :);
    tagnoise = snoise(sidtag, :);
    if stag(i) > 1
        applied = [applied; nanmean(taginput)];
        expected = [expected; nanmean(tagsteady)];
        margin = [margin; max( max(tagsteady + tagnoise) - expected(i, :), ...
                               expected(i, :) - min(tagsteady - tagnoise) )];
    else
        applied = [applied; taginput];
        expected = [expected; tagsteady];
        margin = [margin; tagnoise];
    end
end

%% plot

figure
hold on
for i = 1:size(input, 1)
    errorbar(input(i, :), steady(i, :), noise(i, :));
end
title('Steady - Expanded')
if number > 1
    axis([0 100 0 350])
end

figure
hold on
for i = 1:size(expected, 1)
    errorbar(applied(i, :), expected(i, :), margin(i, :));
end
title('Steady')
if number > 1
    axis([0 100 0 350])
end
% todo: transient plot