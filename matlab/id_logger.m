function data = id_logger(action, varargin)
%ID_LOGGER  Run system-ID experiments and capture the 1 kHz log stream.
%
%  The firmware generates the excitation (bounded, safety-gated) and streams
%  (tick, theta, theta_dot, x, x_dot, u) samples over UDP port 5007 during
%  SYSID and BALANCE runs. This script starts runs and captures the stream.
%
%  Usage:
%    id_logger('zero')                          % capture upright zero (hold link 1 vertical!)
%    data = id_logger('free',  'dur', 15)       % free swing: axis unpowered, just log
%    data = id_logger('chirp', 'amp', 0.2, 'dur', 20, 'f0', 0.2, 'f1', 15)
%    data = id_logger('prbs',  'amp', 0.15, 'dur', 20, 'f1', 10)
%    data = id_logger('step',  'amp', 0.1, 'dur', 2)
%    data = id_logger('listen','dur', 30)       % capture only (e.g. a BALANCE run)
%    id_logger('stop')                          % abort: mode -> IDLE
%
%  Notes:
%    - Driving runs (chirp/prbs/step) need a HOMED rail; 'free' does not.
%    - Firmware re-clamps amp <= 0.6 Nm and dur <= 60 s (sysid.h) no matter
%      what is passed here.
%    - Each capture is saved to id_<type>_YYYYMMDD_HHMMSS.mat next to this
%      file and returned as a struct of column vectors.

%% ---- Config (matches firmware) -----------------------------------------
STM32_IP   = '192.168.1.10';
TX_PORT    = 5005;                       % matlab_rx.c command port
LOG_PORT   = 5007;                       % idlog_tx.c stream port
CMD_MAGIC  = uint32(hex2dec('44434D50'));    % 'PCMD'
LOG_MAGIC  = uint32(hex2dec('474C4449'));    % 'IDLG'
LOOP_HZ    = 5000;                       % M7 tick rate (sample.tick units)
SAMPLE_B   = 24;                         % bytes per wire sample
HDR_B      = 12;                         % bytes per packet header

TYPES = struct('free', 0, 'chirp', 1, 'prbs', 2, 'step', 3);

%% ---- Parse args ---------------------------------------------------------
p = inputParser;
p.addParameter('amp', 0.2,  @(v) isnumeric(v) && v >= 0);
p.addParameter('dur', 10.0, @(v) isnumeric(v) && v > 0);
p.addParameter('f0',  0.2,  @isnumeric);
p.addParameter('f1',  15.0, @isnumeric);
p.parse(varargin{:});
a = p.Results;

action = lower(string(action));
u_tx = udpport("byte");
cleanup_tx = onCleanup(@() clear('u_tx'));

%% ---- Command-only actions ----------------------------------------------
if action == "zero"
    hdr = [typecast(CMD_MAGIC,'uint8'), uint8(8), uint8(0), uint8(0), uint8(0)];
    write(u_tx, hdr, STM32_IP, TX_PORT);
    fprintf('→ ZERO_UPRIGHT sent (link 1 held vertical, right?)\n');
    data = [];
    return
elseif action == "stop"
    hdr = [typecast(CMD_MAGIC,'uint8'), uint8(1), uint8(0), uint8(0), uint8(0)];
    write(u_tx, [hdr, uint8(0)], STM32_IP, TX_PORT);   % SET_MODE IDLE
    fprintf('→ SET_MODE IDLE sent\n');
    data = [];
    return
end

%% ---- Open the log stream BEFORE starting the run ------------------------
u_log = udpport("datagram", "LocalPort", LOG_PORT, "EnablePortSharing", true);
cleanup_log = onCleanup(@() clear('u_log'));
flush(u_log);

%% ---- Start the run ------------------------------------------------------
if action ~= "listen"
    if ~isfield(TYPES, action)
        error('id_logger:badAction', ...
              'Unknown action "%s" (free|chirp|prbs|step|listen|zero|stop)', action);
    end
    typ = TYPES.(char(action));
    pkt = [typecast(CMD_MAGIC,'uint8'), uint8(9), uint8(0), uint8(0), uint8(0), ...
           uint8(typ), uint8(0), uint8(0), uint8(0), ...
           typecast(single(a.amp), 'uint8'), ...
           typecast(single(a.dur), 'uint8'), ...
           typecast(single(a.f0),  'uint8'), ...
           typecast(single(a.f1),  'uint8')];
    write(u_tx, pkt, STM32_IP, TX_PORT);
    fprintf('→ SYSID_RUN %s: amp=%.3f Nm  dur=%.1f s  f0=%.2f  f1=%.2f Hz\n', ...
            action, a.amp, a.dur, a.f0, a.f1);
end

%% ---- Capture ------------------------------------------------------------
capture_s = a.dur + 2.0;                 % arm/finish margin
fprintf('Capturing %.1f s on UDP %d ...\n', capture_s, LOG_PORT);

raw   = {};
drops = 0;
t0 = tic;
while toc(t0) < capture_s
    n = u_log.NumDatagramsAvailable;
    if n > 0
        dg = read(u_log, n, "uint8");
        for k = 1:numel(dg)
            b = uint8(dg(k).Data(:))';
            if numel(b) < HDR_B, continue; end
            if typecast(b(1:4), 'uint32') ~= LOG_MAGIC, continue; end
            ns = double(typecast(b(9:10), 'uint16'));
            drops = double(typecast(b(11:12), 'uint16'));    % latest wins
            need = HDR_B + ns * SAMPLE_B;
            if numel(b) < need, continue; end
            raw{end+1} = b(HDR_B+1 : need); %#ok<AGROW>
        end
    else
        pause(0.005);
    end
end

if isempty(raw)
    warning('id_logger:noData', ...
        'No samples received. Is the run active? (driving types need a homed rail; check the GUI mode pill)');
    data = [];
    return
end

%% ---- Parse --------------------------------------------------------------
blob = [raw{:}];
ns   = floor(numel(blob) / SAMPLE_B);
blob = reshape(blob(1 : ns*SAMPLE_B), SAMPLE_B, ns);

tick      = typecast(reshape(blob(1:4,  :), 1, []), 'uint32')';
theta     = double(typecast(reshape(blob(5:8,  :), 1, []), 'single'))';
theta_dot = double(typecast(reshape(blob(9:12, :), 1, []), 'single'))';
x         = double(typecast(reshape(blob(13:16,:), 1, []), 'single'))';
x_dot     = double(typecast(reshape(blob(17:20,:), 1, []), 'single'))';
u         = double(typecast(reshape(blob(21:24,:), 1, []), 'single'))';

t = (double(tick) - double(tick(1))) / LOOP_HZ;

data = struct('t', t, 'theta', theta, 'theta_dot', theta_dot, ...
              'x', x, 'x_dot', x_dot, 'u', u, ...
              'meta', struct('action', char(action), 'amp', a.amp, ...
                             'dur', a.dur, 'f0', a.f0, 'f1', a.f1, ...
                             'drops', drops, 'captured', datetime('now')));

%% ---- Save + quick look --------------------------------------------------
fname = sprintf('id_%s_%s.mat', char(action), ...
                char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
fdir  = fileparts(mfilename('fullpath'));
save(fullfile(fdir, fname), '-struct', 'data');
fprintf('Saved %d samples (%.1f s, %d ring drops) -> %s\n', ...
        ns, t(end), drops, fname);

figure('Name', sprintf('ID run: %s', action), 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile;
plot(t, rad2deg(theta), 'LineWidth', 1); grid on;
ylabel('\theta  [deg]'); title(sprintf('%s  (amp %.2f Nm)', action, a.amp));
nexttile;
yyaxis left;  plot(t, x, 'LineWidth', 1);      ylabel('x  [rev]');
yyaxis right; plot(t, x_dot, 'LineWidth', 1);  ylabel('dx/dt  [rev/s]');
grid on;
nexttile;
plot(t, u, 'LineWidth', 1); grid on;
ylabel('u  [Nm]'); xlabel('t  [s]');

end
