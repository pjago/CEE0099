function neurose (itag, folder, COM, Gz)
if ~exist(folder, 'dir')
    mkdir(folder)
end
session = dir(folder);
session = session([session.isdir]);
next = session(end).name(end);
if isnan(next) || session(end).name(1) ~= itag
    next = 'a';
elseif length(dir([folder '/' session(end).name])) == 16
    next = next + 1;
elseif length(dir([folder '/' session(end).name])) > 2
    rmdir([folder '/' session(end).name], 's');
end
if ~exist([folder '/' itag next], 'dir')
    mkdir([folder '/' itag next])
end

%% 2 points of operation: (300 +- 10)  e (230 +- 10)

eval([folder '(30, COM, Gz)']) % 107.03 +- 1.09
eval([folder '(40, COM, Gz)']) % 153.47 +- 1.32
eval([folder '(50, COM, Gz)']) % 200.06 +- 1.37
eval([folder '(60, COM, Gz)']) % 241.44 +- 2.46
eval([folder '(70, COM, Gz)']) % 281.56 +- 2.31
eval([folder '(80, COM, Gz)']) % 314.59 +- 2.30
eval([folder '(90, COM, Gz)']) % 341.09 +- 2.04

session = dir(folder);
session = session([session.isdir]);

%% Sythesis

% iterate the steady value twice
steady = nan(length(session)-2, 7);
input = nan(length(session)-2, 7);
noise = nan(length(session)-2, 7);
for i = 1:2
    % in each session the work was to increment the input
    for s = 3:length(session) % start from 3 to skip \. and \..
        work = dir([folder '/' session(s).name '/*.mat']);
        % in each work the steady value should be reached midway through
        for w = 1:length(work)
            load([folder '/' session(s).name '/' work(w).name])
            tag(s-2, w) = nanmean(diff(t));  % todo: find a better tag
            % steady
            if i == 1
                half(s-2, w) = floor(length(t)/2);
            else
                half(s-2, w) = floor(tset5(s-2, w)/nanmean(diff(t))) + 1;
            end
            input(s-2, w) = nanmean(pwm(half(w):end));
            steady(s-2, w) = nanmean(y(half(w):end));
            noise(s-2, w) = std(y(half(w):end));
            % settling
            flipy = flipud(y)/steady(s-2, w);
            downset = find(flipy < 0.95, 1);
            upset = find(flipy > 1.05, 1);
            if isempty(upset)
               tset5(s-2, w) = t(length(t) - downset + 1);
            else
               tset5(s-2, w) = t(length(t) - min(downset, upset) + 1);
            end
            % rise
            gorise = find(y - 0.1*steady(s-2, w) > 0, 1);
            endrise = find(y - 0.9*steady(s-2, w) > 0, 1);
            trise(s-2, w) = t(endrise - gorise + 1);
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

%% Analysis

number = str2double(itag); % of different plants
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

% figure
% hold on
% for i = 1:size(input, 1)
%     errorbar(input(i, :), steady(i, :), noise(i, :));
% end
% title('Steady - Expanded')
% axis([0 100 0 350])
% 
% figure
% hold on
% for i = 1:size(expected, 1)
%     errorbar(applied(i, :), expected(i, :), margin(i, :));
% end
% title('Steady')
% axis([0 100 0 350])

% todo: transient plot