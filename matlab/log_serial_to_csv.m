function log_serial_to_csv(port, outFile, seconds, cmd)
%LOG_SERIAL_TO_CSV  Save the Arduino's CSV stream to a file (MATLAB R2019b+).
%   log_serial_to_csv("COM5", "../data/chirp_run1.csv", 110)
%   log_serial_to_csv("COM5", "../data/controller_steps.csv", 95, "s")
%     the optional 4th argument is sent to the sketch as soon as its header
%     line arrives (e.g. "s" = start the automatic set-point steps).
%   Close the Arduino IDE Serial Monitor first (only one program can own the
%   port). Opening the port resets the Arduino, so the sketch starts again
%   and its header line is captured. Logging stops at "#END" or after
%   `seconds`. Press Ctrl+C to stop early; the file is still closed properly.
if nargin < 3, seconds = 120; end
if nargin < 4, cmd = ""; end
sp = serialport(port, 115200);
configureTerminator(sp, "CR/LF");
sp.Timeout = 5;
flush(sp);
fid = fopen(outFile, 'w');
closer = onCleanup(@() fclose(fid));
fprintf('Logging %s -> %s for up to %g s ...\n', port, outFile, seconds);
t0 = tic;  n = 0;
while toc(t0) < seconds
    line = readline(sp);
    if isempty(line) || any(ismissing(line)), continue; end
    line = strtrim(char(line));
    fprintf(fid, '%s\n', line);
    n = n + 1;
    if strlength(string(cmd)) > 0 && startsWith(line, 't_ms')
        writeline(sp, cmd);
        fprintf('sent "%s" to the Arduino\n', cmd);
    end
    if mod(n, 40) == 0, fprintf('%6.1f s  %s\n', toc(t0), line); end
    if startsWith(line, '#END'), break; end
end
clear sp
fprintf('Saved %d lines to %s\n', n, outFile);
end
