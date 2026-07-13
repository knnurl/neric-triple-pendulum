%% pendulum_monitor.m
%
%  Full MATLAB telemetry monitor + parameter injection for the H755 pendulum.
%
%  USAGE
%  -----
%    Run this script. A live data table prints at 10 Hz. Press Ctrl+C to stop.
%    To send commands while the loop runs, call the helper functions below in
%    the MATLAB command window (they share the UDP object via global vars):
%
%      pm_set_mode(4)             % POT_POSITION
%      pm_set_mode(5)             % POT_VELOCITY
%      pm_set_mode(0)             % IDLE
%      pm_set_setpoint(1.5)       % setpoint 1.5 rev (or rad/s in vel mode)
%      pm_lqr_delta(2, 0.05)      % bump gain index 2 by +0.05
%      pm_clear_errors()
%
%  NETWORK
%  -------
%    STM32 IP  : 192.168.1.10 (fixed, DHCP off)
%    This host : 192.168.1.5  (must match MATLAB_REMOTE_IP_* in telemetry.h)
%    RX port   : 5006  — STM32 sends telemetry here
%    TX port   : 5005  — STM32 listens for commands here
%
%  MODE CODES
%  ----------
%    0 = IDLE          3 = SAFE_STOP
%    1 = SWINGUP       4 = POT_POSITION
%    2 = BALANCE       5 = POT_VELOCITY
%
% -----------------------------------------------------------------------

%% ---- Config -----------------------------------------------------------
STM32_IP    = '192.168.1.10';
RX_PORT     = 5006;
TX_PORT     = 5005;
TELE_MAGIC  = uint32(hex2dec('54454C45'));  % 'TELE'
CMD_MAGIC   = uint32(hex2dec('44434D50'));  % 'PCMD'
PKT_SIZE    = 76;                           % TelemetryPacket_t is 76 bytes
PRINT_HZ    = 10;                           % console refresh rate
HIST_LEN    = 500;                          % rolling sample history

%% ---- Mode name lookup -------------------------------------------------
MODE_NAMES = {'IDLE','SWINGUP','BALANCE','SAFE_STOP','POT_POS','POT_VEL'};

%% ---- Globals shared with helper functions ----------------------------
global PM_UDP_TX PM_STM32_IP PM_TX_PORT PM_CMD_MAGIC;
PM_STM32_IP = STM32_IP;
PM_TX_PORT  = TX_PORT;
PM_CMD_MAGIC = CMD_MAGIC;

%% ---- Open UDP sockets -------------------------------------------------
if exist('u_rx', 'var') && isvalid(u_rx), clear u_rx; end
u_rx = udpport("datagram", "LocalPort", RX_PORT, "EnablePortSharing", true);

PM_UDP_TX = udpport("byte");   % outbound TX (no fixed local port needed)

fprintf('\n========== Pendulum Monitor ==========\n');
fprintf('Telemetry  : listening on port %d\n', RX_PORT);
fprintf('Commands   : sending  to  %s:%d\n', STM32_IP, TX_PORT);
fprintf('Press Ctrl+C to stop.\n\n');

%% ---- Rolling history buffers -----------------------------------------
h.seq        = nan(1, HIST_LEN);
h.ts         = nan(1, HIST_LEN);
h.angle      = nan(3, HIST_LEN);
h.cart_pos   = nan(1, HIST_LEN);
h.cart_vel   = nan(1, HIST_LEN);
h.motor_cmd  = nan(1, HIST_LEN);
h.odrv_pos   = nan(1, HIST_LEN);
h.hb         = nan(1, HIST_LEN);
h.overrun    = nan(1, HIST_LEN);
h.m7_fault   = nan(1, HIST_LEN);
h.m4_fault   = nan(1, HIST_LEN);
h.ctrl_mode  = nan(1, HIST_LEN);
h.cmd_mode   = nan(1, HIST_LEN);
widx         = 0;   % write index into history

%% ---- Stats ------------------------------------------------------------
stats.rx_ok     = 0;
stats.rx_bad    = 0;
stats.dropout   = 0;
stats.last_seq  = 0;
stats.t_start   = tic;
t_last_print    = tic;

%% ---- Main loop --------------------------------------------------------
while true
    n = u_rx.NumDatagramsAvailable;
    if n > 0
        dg = read(u_rx, n, "uint8");
        for k = 1:numel(dg)
            b = uint8(dg(k).Data);
            if numel(b) ~= PKT_SIZE
                stats.rx_bad = stats.rx_bad + 1;
                continue
            end

            % --- magic ---
            magic = typecast(b(1:4), 'uint32');
            if magic ~= TELE_MAGIC
                stats.rx_bad = stats.rx_bad + 1;
                continue
            end

            % --- parse ---
            p.seq        = typecast(b(5:8),   'uint32');
            p.ts         = typecast(b(9:12),  'uint32');
            p.angle      = typecast(b(13:24), 'single');   % [3]
            p.cart_pos   = typecast(b(25:28), 'single');
            p.cart_vel   = typecast(b(29:32), 'single');
            p.motor_cmd  = typecast(b(33:36), 'single');
            p.odrv_pos   = typecast(b(37:40), 'single');
            p.m7_hb      = typecast(b(41:44), 'uint32');
            p.loop_cnt   = typecast(b(45:48), 'uint32');
            p.overrun    = typecast(b(49:52), 'uint32');
            p.m7_fault   = typecast(b(53:56), 'uint32');
            p.m4_fault   = typecast(b(57:60), 'uint32');
            p.ctrl_mode  = b(61);
            p.cmd_mode   = b(62);
            p.sine       = typecast(b(65:68), 'single');
            p.ramp       = typecast(b(69:72), 'single');
            p.counter    = typecast(b(73:76), 'uint32');

            % --- dropout detection ---
            if stats.rx_ok > 0
                gap = double(p.seq) - double(stats.last_seq);
                if gap > 1 && gap < 1000
                    stats.dropout = stats.dropout + (gap - 1);
                end
            end
            stats.last_seq = p.seq;
            stats.rx_ok    = stats.rx_ok + 1;

            % --- store in ring buffer ---
            widx = mod(widx, HIST_LEN) + 1;
            h.seq(widx)       = p.seq;
            h.ts(widx)        = p.ts;
            h.angle(:, widx)  = p.angle(:);
            h.cart_pos(widx)  = p.cart_pos;
            h.cart_vel(widx)  = p.cart_vel;
            h.motor_cmd(widx) = p.motor_cmd;
            h.odrv_pos(widx)  = p.odrv_pos;
            h.hb(widx)        = p.m7_hb;
            h.overrun(widx)   = p.overrun;
            h.m7_fault(widx)  = p.m7_fault;
            h.m4_fault(widx)  = p.m4_fault;
            h.ctrl_mode(widx) = p.ctrl_mode;
            h.cmd_mode(widx)  = p.cmd_mode;
        end
    end

    % --- print at PRINT_HZ ---
    if toc(t_last_print) >= 1/PRINT_HZ && stats.rx_ok > 0
        t_last_print = tic;
        p = struct();
        p.seq       = h.seq(widx);
        p.ts        = h.ts(widx);
        p.angle     = h.angle(:, widx);
        p.cart_pos  = h.cart_pos(widx);
        p.cart_vel  = h.cart_vel(widx);
        p.motor_cmd = h.motor_cmd(widx);
        p.odrv_pos  = h.odrv_pos(widx);
        p.m7_hb     = h.hb(widx);
        p.overrun   = h.overrun(widx);
        p.m7_fault  = h.m7_fault(widx);
        p.m4_fault  = h.m4_fault(widx);
        p.ctrl_mode = h.ctrl_mode(widx);
        p.cmd_mode  = h.cmd_mode(widx);

        ctrl_str = 'UNKNOWN';
        if p.ctrl_mode >= 0 && p.ctrl_mode <= 5
            ctrl_str = MODE_NAMES{p.ctrl_mode + 1};
        end
        cmd_str = 'UNKNOWN';
        if p.cmd_mode >= 0 && p.cmd_mode <= 5
            cmd_str = MODE_NAMES{p.cmd_mode + 1};
        end

        clc
        fprintf('========== Pendulum Monitor  [%.1f s] ==========\n', toc(stats.t_start));
        fprintf(' Packets OK: %u  |  Dropped: %u  |  Bad: %u\n', ...
                stats.rx_ok, stats.dropout, stats.rx_bad);
        fprintf('--------------------------------------------------\n');
        fprintf(' seq            : %u\n', p.seq);
        fprintf(' timestamp (ms) : %u\n', p.ts);
        fprintf(' ctrl_mode      : %s (%d)\n', ctrl_str, p.ctrl_mode);
        fprintf(' cmd_mode       : %s (%d)\n', cmd_str,  p.cmd_mode);
        fprintf('--------------------------------------------------\n');
        fprintf(' L1 angle (rad) : %+8.4f   (%+7.2f deg)\n', p.angle(1), rad2deg(p.angle(1)));
        fprintf(' L2 angle (rad) : %+8.4f   (%+7.2f deg)\n', p.angle(2), rad2deg(p.angle(2)));
        fprintf(' L3 angle (rad) : %+8.4f   (%+7.2f deg)\n', p.angle(3), rad2deg(p.angle(3)));
        fprintf(' cart pos  (rev): %+8.4f\n', p.cart_pos);
        fprintf(' cart vel  (r/s): %+8.4f\n', p.cart_vel);
        fprintf(' motor cmd      : %+8.4f\n', p.motor_cmd);
        fprintf(' ODrive pos(rev): %+8.4f\n', p.odrv_pos);
        fprintf('--------------------------------------------------\n');
        fprintf(' M7 heartbeat   : %u\n', p.m7_hb);
        fprintf(' loop overruns  : %u\n', p.overrun);
        if p.m7_fault ~= 0
            fprintf(' M7 faults      : 0x%08X  *** CHECK! ***\n', p.m7_fault);
        else
            fprintf(' M7 faults      : none\n');
        end
        if p.m4_fault ~= 0
            fprintf(' M4 faults      : 0x%08X  *** CHECK! ***\n', p.m4_fault);
        else
            fprintf(' M4 faults      : none\n');
        end
        fprintf('==================================================\n');
        fprintf(' Commands: pm_set_mode(n)  pm_set_setpoint(v)\n');
        fprintf('           pm_lqr_delta(idx,d)  pm_clear_errors()\n');
    end

    pause(0.002);  % ~500 Hz poll, avoids busy-waiting
end


%% ========== Helper functions ===========================================
%  Call these from the MATLAB command window while the loop is running
%  OR after stopping it (the u_tx socket persists).

function pm_set_mode(mode)
% pm_set_mode(m)  — request controller mode m (0..5)
    global PM_UDP_TX PM_STM32_IP PM_TX_PORT PM_CMD_MAGIC;
    hdr = [typecast(PM_CMD_MAGIC, 'uint8'), uint8(1), uint8(0), uint8(0), uint8(0)];
    pkt = [hdr, uint8(mode)];
    write(PM_UDP_TX, pkt, PM_STM32_IP, PM_TX_PORT);
    fprintf('[TX] SET_MODE -> %d\n', mode);
end

function pm_set_setpoint(sp)
% pm_set_setpoint(v)  — send setpoint v (float, rev or rev/s)
    global PM_UDP_TX PM_STM32_IP PM_TX_PORT PM_CMD_MAGIC;
    hdr = [typecast(PM_CMD_MAGIC, 'uint8'), uint8(2), uint8(0), uint8(0), uint8(0)];
    pkt = [hdr, typecast(single(sp), 'uint8')];
    write(PM_UDP_TX, pkt, PM_STM32_IP, PM_TX_PORT);
    fprintf('[TX] SET_SETPOINT -> %.4f\n', sp);
end

function pm_lqr_delta(idx, delta)
% pm_lqr_delta(i, d)  — bump LQR gain index i (0..5) by d
    global PM_UDP_TX PM_STM32_IP PM_TX_PORT PM_CMD_MAGIC;
    hdr = [typecast(PM_CMD_MAGIC, 'uint8'), uint8(3), uint8(0), uint8(0), uint8(0)];
    pkt = [hdr, uint8(idx), uint8(0), uint8(0), uint8(0), typecast(single(delta), 'uint8')];
    write(PM_UDP_TX, pkt, PM_STM32_IP, PM_TX_PORT);
    fprintf('[TX] LQR_DELTA K[%d] += %.4f\n', idx, delta);
end

function pm_clear_errors()
% pm_clear_errors()  — clear M4 fault flags
    global PM_UDP_TX PM_STM32_IP PM_TX_PORT PM_CMD_MAGIC;
    hdr = [typecast(PM_CMD_MAGIC, 'uint8'), uint8(4), uint8(0), uint8(0), uint8(0)];
    write(PM_UDP_TX, hdr, PM_STM32_IP, PM_TX_PORT);
    fprintf('[TX] CLEAR_ERRORS\n');
end
