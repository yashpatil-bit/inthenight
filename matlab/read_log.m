function D = read_log(file)
%READ_LOG  Read a CSV log written by the Part 2 Arduino sketches.
%   D = read_log(file) returns a struct with one field per CSV column
%   (time_ms, servo_deg, dist_cm, ... or t_ms, volt, ...) plus D.t in s from 0.
%   Lines that are not numeric (comments, "#END", partial lines) are skipped.
txt   = fileread(file);
lines = regexp(txt, '\r?\n', 'split');
hdr   = find(~cellfun(@isempty, regexp(lines, '^(t_ms|time_ms),')), 1);
if isempty(hdr)
    error('read_log:noHeader', 'No "time_ms,..." / "t_ms,..." header in %s', file);
end
names = strtrim(strsplit(lines{hdr}, ','));
nc    = numel(names);
X     = nan(numel(lines), nc);
k     = 0;
for i = hdr+1:numel(lines)
    v = sscanf(lines{i}, '%f,');
    if numel(v) == nc
        k = k + 1;
        X(k, :) = v(:).';
    end
end
X = X(1:k, :);
for j = 1:nc
    D.(names{j}) = X(:, j);
end
if isfield(D, 'time_ms'), D.t_ms = D.time_ms; end
D.t = (D.t_ms - D.t_ms(1)) / 1000;
fprintf('read_log: %d samples (%.1f s) from %s\n', k, D.t(end), file);
end
