function pendulum_gui()
%PENDULUM_GUI  Live telemetry + command dashboard for the H755 pendulum.
%
%  Run:  pendulum_gui()
%
%  A non-blocking dashboard: a live link-angle plot, KPI tiles, status pills
%  and a control column. The MATLAB command window stays free while it is up.
%
%  Network:
%    STM32 IP  : 192.168.1.10  (must match MATLAB_REMOTE_IP_* in telemetry.h)
%    RX port   : 5006  (STM32 sends telemetry here)
%    TX port   : 5005  (STM32 listens for commands here)

%% ---- Config -----------------------------------------------------------
STM32_IP    = '192.168.1.10';
LOCAL_IP    = '192.168.1.5';   % this PC's address on the STM32 link (telemetry.h)
RX_PORT     = 5006;
TX_PORT     = 5005;
TELE_MAGIC  = uint32(hex2dec('54454C45'));
CMD_MAGIC   = uint32(hex2dec('44434D50'));
PKT_SIZE    = 106;     % 76-byte base + 30-byte encoder-diagnostics block
POLL_MS     = 50;      % GUI refresh interval (ms)
RAIL_TRAVEL = 12;      % rev, home switch -> far end (= RAIL_TRAVEL_REV in rail_limits.h)
WIN         = 300;     % plot history length (samples) -> ~15 s at 50 ms

%% ---- Theme ------------------------------------------------------------
C.bg      = [0.075 0.086 0.110];   % window
C.card    = [0.130 0.145 0.185];   % card / panel
C.tile    = [0.165 0.180 0.225];   % tile / pill
C.text    = [0.920 0.940 0.970];
C.dim     = [0.520 0.560 0.640];
C.good    = [0.320 0.800 0.480];
C.warn    = [0.980 0.740 0.220];
C.bad     = [0.950 0.360 0.360];
C.accent  = [0.290 0.640 0.960];
C.btn     = [0.220 0.240 0.300];
C.line    = [0.30 0.32 0.38];      % subtle separators / grid

FONT = 'Segoe UI';
MONO = 'Consolas';

% Series colours (link angles) — distinct + accessible on dark.
COL_L1 = [0.29 0.64 0.96];
COL_L2 = [0.98 0.74 0.22];
COL_L3 = [0.40 0.82 0.52];

% 7th entry (SYSID) is display-only — runs start via id_logger.m, not a button.
MODE_NAMES  = {'IDLE','SWING-UP','BALANCE','E-STOP','POT POS','POT VEL','SYSID'};
MODE_COLORS = {[0.40 0.42 0.50], [0.85 0.55 0.15], [0.20 0.70 0.38], ...
               [0.85 0.22 0.22], [0.25 0.55 0.85], [0.16 0.62 0.62], ...
               [0.65 0.45 0.85]};

%% ---- Open UDP sockets -------------------------------------------------
u_rx = udpport("datagram", "LocalPort", RX_PORT, "EnablePortSharing", true);
% Pin the command socket to the NIC that owns LOCAL_IP. On a PC with more
% than one adapter in 192.168.1.0/24 (lab Ethernet + WiFi on a home router),
% an unpinned socket can route commands out the WRONG adapter: telemetry
% still arrives (u_rx binds all interfaces) but nothing ever reaches the
% STM32 — the "REMOTE snaps back / CMD SEQ stays 0" symptom.
try
    u_tx = udpport("byte", "LocalHost", LOCAL_IP);
catch
    warning('pendulum_gui:txbind', ...
        'Could not bind TX socket to %s — is that this PC''s address on the STM32 link? Falling back to OS routing.', LOCAL_IP);
    u_tx = udpport("byte");
end

%% ---- App state --------------------------------------------------------
s = struct();
s.rx_ok    = 0;
s.rx_bad   = 0;
s.dropout  = 0;
s.last_seq = 0;
s.enc_err_prev = [0 0 0];   % per-encoder reject totals, for delta detection
s.fault_tt_m7  = uint32(2^32-1);   % last tooltip fault words (force first build)
s.fault_tt_m4  = uint32(2^32-1);
s.fault_prev   = false;            % edge-detect faults to reset the vel slider
s.t_start  = tic;
s.p        = make_empty_packet();
% Plot ring buffers (degrees)
s.thist = (-(WIN-1):0) * (POLL_MS/1000);
s.a1 = nan(1,WIN);
s.a2 = nan(1,WIN);
s.a3 = nan(1,WIN);

%% ---- Figure -----------------------------------------------------------
fig = uifigure( ...
    'Name',     'Pendulum Monitor', ...
    'Position', [80 60 1180 720], ...
    'Color',     C.bg);

root = uigridlayout(fig, [2 1], ...
    'RowHeight',    {54, '1x'}, ...
    'ColumnWidth',  {'1x'}, ...
    'BackgroundColor', C.bg, ...
    'Padding', [12 12 12 12], ...
    'RowSpacing', 10);

%% ======================================================================
%  Small builder helpers (nested — share theme vars)
%  ======================================================================
    function vlbl = tile(parent, caption, valcolor)
        pnl = uipanel(parent, 'BackgroundColor', C.tile, 'BorderType', 'none');
        tg  = uigridlayout(pnl, [2 1], 'RowHeight', {12, '1x'}, ...
            'ColumnWidth', {'1x'}, 'Padding', [11 3 11 3], ...
            'RowSpacing', 1, 'BackgroundColor', C.tile);
        uilabel(tg, 'Text', upper(caption), 'FontSize', 8.5, ...
            'FontColor', C.dim, 'FontName', FONT, 'BackgroundColor', 'none');
        vlbl = uilabel(tg, 'Text', '—', 'FontSize', 16, 'FontWeight', 'bold', ...
            'FontColor', valcolor, 'FontName', MONO, 'BackgroundColor', 'none', ...
            'VerticalAlignment', 'center');
        vlbl.Layout.Row = 2;
    end

    function [lmp, vlbl] = statpill(parent, caption)
        pnl = uipanel(parent, 'BackgroundColor', C.tile, 'BorderType', 'none');
        gg  = uigridlayout(pnl, [2 1], 'RowHeight', {12, '1x'}, ...
            'ColumnWidth', {'1x'}, 'Padding', [11 4 11 4], ...
            'RowSpacing', 1, 'BackgroundColor', C.tile);
        uilabel(gg, 'Text', upper(caption), 'FontSize', 8.5, ...
            'FontColor', C.dim, 'FontName', FONT, 'BackgroundColor', 'none');
        rg = uigridlayout(gg, [1 2], 'RowHeight', {'1x'}, ...
            'ColumnWidth', {16, '1x'}, 'Padding', [0 0 0 0], ...
            'ColumnSpacing', 7, 'BackgroundColor', C.tile);
        rg.Layout.Row = 2;
        lmp = uilamp(rg, 'Color', C.dim);
        vlbl = uilabel(rg, 'Text', '—', 'FontSize', 13.5, 'FontWeight', 'bold', ...
            'FontColor', C.text, 'FontName', FONT, 'BackgroundColor', 'none', ...
            'VerticalAlignment', 'center');
        vlbl.Layout.Column = 2;
    end

    function h = section(parent, txt)
        h = uilabel(parent, 'Text', upper(txt), 'FontSize', 9.5, ...
            'FontWeight', 'bold', 'FontColor', C.dim, 'FontName', FONT, ...
            'BackgroundColor', 'none', 'VerticalAlignment', 'center');
    end

    function mkquick(parent, r, c, txt, cb)
        b = uibutton(parent, 'Text', txt, 'FontSize', 9, 'FontName', FONT, ...
            'BackgroundColor', C.btn, 'FontColor', C.text, ...
            'ButtonPushedFcn', @(~,~) cb());
        b.Layout.Row = r;  b.Layout.Column = c;
    end

%% ======================================================================
%  Header bar
%  ======================================================================
hdr_pnl = uipanel(root, 'BackgroundColor', C.card, 'BorderType', 'none');
hdr_pnl.Layout.Row = 1;
hdr = uigridlayout(hdr_pnl, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x', 360}, 'Padding', [16 6 16 6], ...
    'BackgroundColor', C.card);

uilabel(hdr, 'Text', '⬤  Triple Pendulum — Live Monitor', ...
    'FontSize', 16, 'FontWeight', 'bold', 'FontName', FONT, ...
    'FontColor', C.accent, 'BackgroundColor', 'none', ...
    'VerticalAlignment', 'center');

lbl_conn = uilabel(hdr, 'Text', 'waiting for telemetry…', ...
    'FontSize', 11, 'FontName', MONO, 'FontColor', C.dim, ...
    'BackgroundColor', 'none', 'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'center');
lbl_conn.Layout.Column = 2;

%% ======================================================================
%  Body: plot + telemetry (left)  |  controls (right)
%  ======================================================================
body = uigridlayout(root, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x', 336}, 'BackgroundColor', C.bg, ...
    'Padding', [0 0 0 0], 'ColumnSpacing', 10);
body.Layout.Row = 2;

% ---- LEFT column: status pills / plot / tiles -------------------------
left = uigridlayout(body, [3 1], 'RowHeight', {62, '1x', 200}, ...
    'ColumnWidth', {'1x'}, 'BackgroundColor', C.bg, ...
    'Padding', [0 0 0 0], 'RowSpacing', 10);
left.Layout.Column = 1;

% -- Status pills --
pills = uigridlayout(left, [1 4], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x','1x','1x','1x'}, 'BackgroundColor', C.bg, ...
    'Padding', [0 0 0 0], 'ColumnSpacing', 10);
pills.Layout.Row = 1;
[lmp_mode,  val_mode]  = statpill(pills, 'Mode');
[lmp_rail,  val_rail]  = statpill(pills, 'Rail');
[lmp_fault, val_fault] = statpill(pills, 'Faults M7 / M4');
[lmp_link,  val_link]  = statpill(pills, 'Link');
val_mode.Text = 'IDLE';  val_rail.Text = 'UNHOMED';
val_fault.Text = 'none'; val_link.Text = 'offline';

% -- Live angle plot --
plot_pnl = uipanel(left, 'BackgroundColor', C.card, 'BorderType', 'none');
plot_pnl.Layout.Row = 2;
plot_grid = uigridlayout(plot_pnl, [2 1], 'RowHeight', {18, '1x'}, ...
    'ColumnWidth', {'1x'}, 'Padding', [12 8 12 10], ...
    'RowSpacing', 4, 'BackgroundColor', C.card);
section(plot_grid, 'Link angles (deg)');

ax = uiaxes(plot_grid);
ax.Layout.Row = 2;
ax.Color = C.card;
ax.XColor = C.dim;  ax.YColor = C.dim;
ax.GridColor = [1 1 1];  ax.GridAlpha = 0.08;
ax.XGrid = 'on';  ax.YGrid = 'on';
ax.Box = 'off';
ax.FontSize = 8.5;  ax.FontName = MONO;
ax.XLim = [s.thist(1) 0];
ax.YLim = [-10 10];  ax.YLimMode = 'auto';
ax.XLabel.String = 'seconds ago';  ax.XLabel.Color = C.dim;
hold(ax, 'on');
ln1 = plot(ax, s.thist, s.a1, 'LineWidth', 1.7, 'Color', COL_L1);
ln2 = plot(ax, s.thist, s.a2, 'LineWidth', 1.7, 'Color', COL_L2);
ln3 = plot(ax, s.thist, s.a3, 'LineWidth', 1.7, 'Color', COL_L3);
lg = legend(ax, {'L1','L2','L3'}, 'Orientation', 'horizontal', ...
    'Location', 'northwest');
lg.TextColor = C.text;  lg.Color = C.tile;  lg.Box = 'off';
lg.FontSize = 9;

% -- KPI tiles --
tile_pnl = uipanel(left, 'BackgroundColor', C.card, 'BorderType', 'none');
tile_pnl.Layout.Row = 3;
tile_grid = uigridlayout(tile_pnl, [2 1], 'RowHeight', {18, '1x'}, ...
    'ColumnWidth', {'1x'}, 'Padding', [12 8 12 10], ...
    'RowSpacing', 6, 'BackgroundColor', C.card);
section(tile_grid, 'Telemetry');
tiles = uigridlayout(tile_grid, [3 4], 'RowHeight', {'1x','1x','1x'}, ...
    'ColumnWidth', {'1x','1x','1x','1x'}, 'Padding', [0 0 0 0], ...
    'RowSpacing', 8, 'ColumnSpacing', 8, 'BackgroundColor', C.card);
tiles.Layout.Row = 2;
tl_L1    = tile(tiles, 'L1 angle',   COL_L1);
tl_L2    = tile(tiles, 'L2 angle',   COL_L2);
tl_L3    = tile(tiles, 'L3 angle',   COL_L3);
tl_cartp = tile(tiles, 'Cart pos',   C.text);
tl_cartv = tile(tiles, 'Cart vel',   C.text);
tl_motor = tile(tiles, 'Motor cmd',  C.text);
tl_odrv  = tile(tiles, 'ODrive pos', C.text);
tl_cseq  = tile(tiles, 'CMD seq',    C.dim);
% Row 3: per-encoder sensor health — raw SPI word · AGC, coloured by state
% (red = dead bus / parity storm, amber = field out of range, green = OK).
tl_enc = gobjects(1,3);
tl_enc(1) = tile(tiles, 'L1 sensor', C.dim);
tl_enc(2) = tile(tiles, 'L2 sensor', C.dim);
tl_enc(3) = tile(tiles, 'L3 sensor', C.dim);
tl_encerr = tile(tiles, 'Enc errs P/E/D', C.dim);

% ---- RIGHT column: controls card --------------------------------------
ctrl_pnl = uipanel(body, 'BackgroundColor', C.card, 'BorderType', 'none');
ctrl_pnl.Layout.Column = 2;
ctrl = uigridlayout(ctrl_pnl, [12 1], ...
    'RowHeight', {16, 108, 15, 74, 15, 74, 15, 60, 34, 34, 16, '1x'}, ...
    'ColumnWidth', {'1x'}, 'BackgroundColor', C.card, ...
    'Padding', [14 12 14 12], 'RowSpacing', 6);

% -- Mode --
sh = section(ctrl, 'Mode');  sh.Layout.Row = 1;
mode_grid = uigridlayout(ctrl, [3 2], 'RowHeight', {'1x','1x','1x'}, ...
    'ColumnWidth', {'1x','1x'}, 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'RowSpacing', 5, 'ColumnSpacing', 5);
mode_grid.Layout.Row = 2;
mode_btns = gobjects(1, 6);
for i = 1:6
    r = ceil(i/2);  c = mod(i-1,2)+1;
    mode_btns(i) = uibutton(mode_grid, 'Text', MODE_NAMES{i}, ...
        'FontSize', 11, 'FontWeight', 'bold', 'FontName', FONT, ...
        'FontColor', C.text, 'BackgroundColor', C.btn, ...
        'ButtonPushedFcn', @(~,~) send_mode(i-1));
    mode_btns(i).Layout.Row = r;  mode_btns(i).Layout.Column = c;
end
mode_btns(4).BackgroundColor = [0.55 0.12 0.12];   % E-STOP always red

% -- Position setpoint --
pos_hdr = uigridlayout(ctrl, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x', 90}, 'Padding', [0 0 0 0], 'BackgroundColor', C.card);
pos_hdr.Layout.Row = 3;
section(pos_hdr, 'Position  (rev · Home first)');
lbl_pos_val = uilabel(pos_hdr, 'Text', '0.000', 'FontSize', 11, ...
    'FontWeight', 'bold', 'FontName', MONO, 'FontColor', C.accent, ...
    'BackgroundColor', 'none', 'HorizontalAlignment', 'right');
lbl_pos_val.Layout.Column = 2;

pos_pnl = uigridlayout(ctrl, [2 3], 'RowHeight', {'1x', 24}, ...
    'ColumnWidth', {'1x','1x','1x'}, 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'RowSpacing', 4, 'ColumnSpacing', 5);
pos_pnl.Layout.Row = 4;
pos_slider = uislider(pos_pnl, 'Limits', [0 RAIL_TRAVEL], 'Value', 0, ...
    'MajorTicks', 0:2:RAIL_TRAVEL, 'FontSize', 8, ...
    'FontColor', C.dim, ...
    'ValueChangingFcn', @on_pos_slide, 'ValueChangedFcn', @on_pos_slide);
pos_slider.Layout.Row = 1;  pos_slider.Layout.Column = [1 3];
mkquick(pos_pnl, 2, 1, 'Home end', @() set_pos_value(0));
mkquick(pos_pnl, 2, 2, 'Mid',      @() set_pos_value(RAIL_TRAVEL/2));
mkquick(pos_pnl, 2, 3, 'Far end',  @() set_pos_value(RAIL_TRAVEL));

% -- Velocity setpoint --
vel_hdr = uigridlayout(ctrl, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x', 90}, 'Padding', [0 0 0 0], 'BackgroundColor', C.card);
vel_hdr.Layout.Row = 5;
section(vel_hdr, 'Velocity  (rev/s)');
lbl_vel_val = uilabel(vel_hdr, 'Text', '0.000', 'FontSize', 11, ...
    'FontWeight', 'bold', 'FontName', MONO, 'FontColor', C.accent, ...
    'BackgroundColor', 'none', 'HorizontalAlignment', 'right');
lbl_vel_val.Layout.Column = 2;

vel_pnl = uigridlayout(ctrl, [2 3], 'RowHeight', {'1x', 24}, ...
    'ColumnWidth', {'1x','1x','1x'}, 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'RowSpacing', 4, 'ColumnSpacing', 5);
vel_pnl.Layout.Row = 6;
vel_slider = uislider(vel_pnl, 'Limits', [-10 10], 'Value', 0, ...
    'MajorTicks', -10:5:10, 'FontSize', 8, 'FontColor', C.dim, ...
    'ValueChangingFcn', @on_vel_slide, 'ValueChangedFcn', @on_vel_slide);
vel_slider.Layout.Row = 1;  vel_slider.Layout.Column = [1 3];
mkquick(vel_pnl, 2, 1, '−1', @() bump_vel_value(-1));
mkquick(vel_pnl, 2, 2, '0',  @() set_vel_value(0));
mkquick(vel_pnl, 2, 3, '+1', @() bump_vel_value(1));

% -- LQR tuning --
sh = section(ctrl, 'LQR tuning');  sh.Layout.Row = 7;
gain_grid = uigridlayout(ctrl, [2 3], 'RowHeight', {'1x','1x'}, ...
    'ColumnWidth', {74, '1x', '1x'}, 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'ColumnSpacing', 6, 'RowSpacing', 5);
gain_grid.Layout.Row = 8;
uilabel(gain_grid, 'Text', 'Gain idx', 'FontSize', 10, 'FontColor', C.dim, ...
    'FontName', FONT, 'BackgroundColor', 'none', 'VerticalAlignment', 'center');
gain_idx = uispinner(gain_grid, 'Value', 0, 'Step', 1, 'Limits', [0 5], ...
    'ValueDisplayFormat', '%d', 'FontSize', 11);
lbl_gain_idx = uilabel(gain_grid, 'Text', '(K0)', 'FontSize', 10, ...
    'FontColor', C.dim, 'FontName', FONT, 'BackgroundColor', 'none', ...
    'VerticalAlignment', 'center');
uilabel(gain_grid, 'Text', 'Delta', 'FontSize', 10, 'FontColor', C.dim, ...
    'FontName', FONT, 'BackgroundColor', 'none', 'VerticalAlignment', 'center');
gain_delta = uispinner(gain_grid, 'Value', 0.05, 'Step', 0.01, ...
    'Limits', [0.001 1.0], 'ValueDisplayFormat', '%.3f', 'FontSize', 11);
g_btn_row = uigridlayout(gain_grid, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x','1x'}, 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'ColumnSpacing', 5);
uibutton(g_btn_row, 'Text', '+ K', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.20 0.55 0.28], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @(~,~) send_gain_delta(gain_idx.Value, gain_delta.Value));
uibutton(g_btn_row, 'Text', '− K', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.55 0.22 0.22], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @(~,~) send_gain_delta(gain_idx.Value, -gain_delta.Value));

% -- Control source switch (row 9) --
src_row = uigridlayout(ctrl, [1 2], 'RowHeight', {'1x'}, ...
    'ColumnWidth', {'1x', 108}, 'BackgroundColor', C.card, 'Padding', [0 0 0 0]);
src_row.Layout.Row = 9;
uilabel(src_row, 'Text', 'Control source', 'FontSize', 10, 'FontColor', C.dim, ...
    'FontName', FONT, 'BackgroundColor', 'none', 'VerticalAlignment', 'center');
ctrlsrc_switch = uiswitch(src_row, 'slider', 'Items', {'POT','REMOTE'}, ...
    'Value', 'POT', 'FontSize', 9, 'FontColor', C.text, ...
    'ValueChangedFcn', @(src,~) send_ctrl_src(strcmp(src.Value,'REMOTE')));
ctrlsrc_switch.Layout.Column = 2;

% -- Action buttons (row 10): Home + Zero θ + Clear Errors --
% (Index Search removed — the ODrive S1 onboard absolute encoder needs none.)
act_row = uigridlayout(ctrl, [1 3], 'BackgroundColor', C.card, ...
    'Padding', [0 0 0 0], 'ColumnSpacing', 6, ...
    'RowHeight', {'1x'}, 'ColumnWidth', {'1x','1x','1x'});
act_row.Layout.Row = 10;
uibutton(act_row, 'Text', '⌂ Home', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.15 0.45 0.55], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @(~,~) send_home());
uibutton(act_row, 'Text', '⊥ Zero θ', 'FontSize', 10, 'FontWeight', 'bold', ...
    'Tooltip', 'Hold link 1 vertical, then click: captures the upright reference', ...
    'BackgroundColor', [0.35 0.30 0.60], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @(~,~) send_zero_upright());
uibutton(act_row, 'Text', '✕ Clear Errors', 'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.55 0.40 0.10], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @(~,~) send_clear_errors());

% -- Last TX status (row 11) --
lbl_last_tx = uilabel(ctrl, 'Text', '', 'FontSize', 9, 'FontName', MONO, ...
    'FontColor', C.dim, 'BackgroundColor', 'none', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'center');
lbl_last_tx.Layout.Row = 11;

%% ---- Timer ------------------------------------------------------------
tmr = timer('Period', POLL_MS/1000, 'ExecutionMode', 'fixedRate', ...
            'TimerFcn', @poll_and_update);
start(tmr);
fig.CloseRequestFcn = @on_close;

%% ========== Nested callbacks ==========================================

    function poll_and_update(~, ~)
        try
        if ~isvalid(fig), return; end
        n = u_rx.NumDatagramsAvailable;
        if n > 0
            dg = read(u_rx, n, "uint8");
            for k = 1:numel(dg)
                b = uint8(dg(k).Data);
                if numel(b) ~= PKT_SIZE; s.rx_bad = s.rx_bad+1; continue; end
                if typecast(b(1:4),'uint32') ~= TELE_MAGIC; s.rx_bad = s.rx_bad+1; continue; end
                p = parse_packet(b);
                gap = double(p.seq) - double(s.last_seq);
                if s.rx_ok > 0 && gap > 1 && gap < 1000
                    s.dropout = s.dropout + (gap-1);
                end
                s.last_seq = p.seq;
                s.rx_ok = s.rx_ok + 1;
                s.p = p;
            end
        end

        if s.rx_ok == 0, return; end
        p = s.p;

        % ---- Scroll the angle history + refresh plot ----
        s.a1 = [s.a1(2:end), rad2deg(p.angle(1))];
        s.a2 = [s.a2(2:end), rad2deg(p.angle(2))];
        s.a3 = [s.a3(2:end), rad2deg(p.angle(3))];
        ln1.YData = s.a1;  ln2.YData = s.a2;  ln3.YData = s.a3;

        % ---- KPI tiles ----
        tl_L1.Text    = sprintf('%+.1f°', rad2deg(p.angle(1)));
        tl_L2.Text    = sprintf('%+.1f°', rad2deg(p.angle(2)));
        tl_L3.Text    = sprintf('%+.1f°', rad2deg(p.angle(3)));
        tl_cartp.Text = sprintf('%+.3f', p.cart_pos);
        tl_cartv.Text = sprintf('%+.3f', p.cart_vel);
        tl_motor.Text = sprintf('%+.4f', p.motor_cmd);
        tl_odrv.Text  = sprintf('%+.3f', p.odrv_pos);
        if p.command_seq > 0
            tl_cseq.Text = sprintf('%u', p.command_seq);
            tl_cseq.FontColor = C.good;
        else
            tl_cseq.Text = '0';
            tl_cseq.FontColor = C.dim;
        end

        % ---- Encoder sensor-health tiles (row 3) ----
        % raw 0x0000 = dead bus (MISO/power/CS); MAGL/MAGH = magnet field
        % out of range; err counters rising = frames being rejected.
        for ei = 1:3
            agc  = bitand(p.enc_diaagc(ei), 255);
            magh = bitand(p.enc_diaagc(ei), 1024) > 0;
            magl = bitand(p.enc_diaagc(ei), 2048) > 0;
            errs = p.enc_par(ei) + p.enc_ef(ei) + p.enc_dead(ei);
            derr = errs - s.enc_err_prev(ei);          % new errors since last frame
            s.enc_err_prev(ei) = errs;
            tl_enc(ei).Text = sprintf('%04X·A%u', p.enc_raw(ei), agc);
            if p.enc_raw(ei) == 0 || derr > 0
                tl_enc(ei).FontColor = C.bad;          % dead bus / active rejects
            elseif magl || magh
                tl_enc(ei).FontColor = C.warn;         % magnet field out of range
            else
                tl_enc(ei).FontColor = C.good;
            end
        end
        tl_encerr.Text = sprintf('%u/%u/%u', ...
            sum(p.enc_par), sum(p.enc_ef), sum(p.enc_dead));
        if sum(p.enc_par) + sum(p.enc_ef) + sum(p.enc_dead) > 0
            tl_encerr.FontColor = C.warn;
        else
            tl_encerr.FontColor = C.dim;
        end

        % ---- Mode pill ----
        % When the rail is mid-trip or latched the motor is cut (ODrive
        % idle/estop), so the commanded controller_mode is stale. Show IDLE so
        % the mode follows the real stopped state instead of the pre-trip mode.
        if p.rail_state == 3 || p.rail_state == 4   % HARD_TRIP / LATCHED
            eff_mode = 0;                           % CTRL_MODE_IDLE
        else
            eff_mode = double(p.ctrl_mode);
        end
        cm = eff_mode + 1;
        if cm >= 1 && cm <= numel(MODE_NAMES)
            val_mode.Text  = MODE_NAMES{cm};
            lmp_mode.Color = MODE_COLORS{cm};
        end

        % ---- Velocity value label tracks live command in POT_VELOCITY ----
        if p.ctrl_mode == 5
            lbl_vel_val.Text = sprintf('%+.3f', p.motor_cmd);
        end

        % ---- Fault pill ----
        has_fault = (p.m7_fault ~= 0) || (p.m4_fault ~= 0);
        % On the rising edge of any fault (e.g. rail latch / ODrive axis), the
        % motor has been cut — snap the velocity slider back to 0 so it doesn't
        % resume at the old setpoint when the operator clears and re-commands.
        if has_fault && ~s.fault_prev
            vel_slider.Value = 0;
            lbl_vel_val.Text = '0.000';
        end
        s.fault_prev = has_fault;
        if has_fault
            val_fault.Text  = sprintf('%08X / %08X', p.m7_fault, p.m4_fault);
            val_fault.FontColor = C.bad;
            lmp_fault.Color = C.bad;
        else
            val_fault.Text  = 'none / none';
            val_fault.FontColor = C.text;
            lmp_fault.Color = C.good;
        end
        % Hover elaboration: decode each set bit into name + meaning + hint.
        % (Only rebuild the tooltip when the fault words change — tooltip
        % churn at 20 Hz makes hover flicker.)
        if p.m7_fault ~= s.fault_tt_m7 || p.m4_fault ~= s.fault_tt_m4
            s.fault_tt_m7 = p.m7_fault;
            s.fault_tt_m4 = p.m4_fault;
            val_fault.Tooltip = decode_faults_tooltip(p.m7_fault, p.m4_fault);
        end

        % ---- Rail pill ----
        switch p.rail_state
            case 0, val_rail.Text = 'UNHOMED';   lmp_rail.Color = C.warn;
            case 1, val_rail.Text = 'OK';        lmp_rail.Color = C.good;
            case 2, val_rail.Text = 'SOFT LIMIT';lmp_rail.Color = C.warn;
            case 3, val_rail.Text = 'HARD TRIP'; lmp_rail.Color = C.bad;
            case 4, val_rail.Text = 'LATCHED';   lmp_rail.Color = C.bad;
            otherwise, val_rail.Text = sprintf('? %d', p.rail_state); lmp_rail.Color = C.dim;
        end

        % ---- Control-source switch reflects firmware truth ----
        if p.ctrl_src == 0
            ctrlsrc_switch.Value = 'POT';
        else
            ctrlsrc_switch.Value = 'REMOTE';
        end

        % ---- Active mode button highlight (uses eff_mode so a rail trip/latch
        % clears the highlight instead of pinning it to the pre-trip mode) ----
        for i = 1:6
            if eff_mode == (i-1)
                mode_btns(i).BackgroundColor = MODE_COLORS{i};
            else
                mode_btns(i).BackgroundColor = C.btn;
            end
        end
        mode_btns(4).BackgroundColor = [0.55 0.12 0.12];  % E-STOP always red

        % ---- Link pill + header ----
        elapsed = toc(s.t_start);
        rate = s.rx_ok / max(elapsed, 0.001);
        val_link.Text  = sprintf('%.0f pk/s', rate);
        lmp_link.Color = C.good;
        % cmd rx = datagrams the STM32 saw on 5005 (any validity);
        % cmd seq = commands it ACCEPTED. rx frozen => network problem;
        % rx climbing with seq stuck => firmware is rejecting them.
        lbl_conn.Text  = sprintf('%s   %.0f pk/s   ok %u   drop %u   cmd rx %u / seq %u', ...
            STM32_IP, rate, s.rx_ok, s.dropout, ...
            uint32(p.cmd_rx), uint32(p.command_seq));
        lbl_conn.FontColor = C.good;

        % ---- Gain index label ----
        idx_names = {'K0','K1','K2','K3','K4','K5'};
        lbl_gain_idx.Text = sprintf('(%s)', idx_names{gain_idx.Value+1});

        catch ME
            fprintf('[GUI timer error] %s\n', ME.message);
        end
    end

    % ---- Slider callbacks (live: fire on each drag step) ---------------
    % NOTE: read event.Value, not src.Value — for ValueChangingFcn the
    % underlying property only updates on release, event.Value is live.
    function on_pos_slide(~, event)
        v = event.Value;
        lbl_pos_val.Text = sprintf('%+.3f', v);
        send_mode(4);       % CTRL_MODE_POT_POSITION
        send_setpoint(v);
    end

    function on_vel_slide(~, event)
        v = event.Value;
        lbl_vel_val.Text = sprintf('%+.3f', v);
        send_mode(5);       % CTRL_MODE_POT_VELOCITY
        send_setpoint(v);
    end

    function set_pos_value(v)
        pos_slider.Value = v;
        lbl_pos_val.Text = sprintf('%+.3f', v);
        send_mode(4);
        send_setpoint(v);
    end

    function set_vel_value(v)
        vel_slider.Value = v;
        lbl_vel_val.Text = sprintf('%+.3f', v);
        send_mode(5);
        send_setpoint(v);
    end

    function bump_vel_value(delta)
        lim = vel_slider.Limits;
        v = max(lim(1), min(lim(2), vel_slider.Value + delta));
        set_vel_value(v);
    end

    function send_mode(m)
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(1), uint8(0), uint8(0), uint8(0)];
        write(u_tx, [hdr_b, uint8(m)], STM32_IP, TX_PORT);
        lbl_last_tx.Text = sprintf('→ SET_MODE: %s', MODE_NAMES{m+1});
    end

    function send_setpoint(v)
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(2), uint8(0), uint8(0), uint8(0)];
        write(u_tx, [hdr_b, typecast(single(v),'uint8')], STM32_IP, TX_PORT);
        lbl_last_tx.Text = sprintf('→ SET_SETPOINT %.3f', v);
    end

    function send_gain_delta(idx, d)
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(3), uint8(0), uint8(0), uint8(0)];
        pkt = [hdr_b, uint8(idx), uint8(0), uint8(0), uint8(0), typecast(single(d),'uint8')];
        write(u_tx, pkt, STM32_IP, TX_PORT);
        lbl_last_tx.Text = sprintf('→ K%d %+.3f', idx, d);
    end

    function send_clear_errors()
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(4), uint8(0), uint8(0), uint8(0)];
        write(u_tx, hdr_b, STM32_IP, TX_PORT);
        lbl_last_tx.Text = '→ CLEAR_ERRORS';
    end

    function send_ctrl_src(is_remote)
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(6), uint8(0), uint8(0), uint8(0)];
        write(u_tx, [hdr_b, uint8(is_remote)], STM32_IP, TX_PORT);
        if is_remote
            lbl_last_tx.Text = '→ CTRL_SRC: REMOTE';
        else
            lbl_last_tx.Text = '→ CTRL_SRC: POT';
        end
    end

    function send_home()
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(7), uint8(0), uint8(0), uint8(0)];
        write(u_tx, hdr_b, STM32_IP, TX_PORT);
        lbl_last_tx.Text = '→ HOME (rail zero)';
    end

    function send_zero_upright()
        hdr_b = [typecast(CMD_MAGIC,'uint8'), uint8(8), uint8(0), uint8(0), uint8(0)];
        write(u_tx, hdr_b, STM32_IP, TX_PORT);
        lbl_last_tx.Text = '→ ZERO θ (upright captured)';
    end

    function on_close(~, ~)
        if isvalid(tmr), stop(tmr); delete(tmr); end
        if isvalid(u_rx), clear u_rx; end
        if isvalid(u_tx), clear u_tx; end
        delete(fig);
    end

end  % pendulum_gui

%% ---- Utilities (file-scope, not nested) --------------------------------

function p = parse_packet(b)
    p.seq       = typecast(b(5:8),   'uint32');
    p.ts        = typecast(b(9:12),  'uint32');
    p.angle     = double(typecast(b(13:24), 'single'));
    p.cart_pos  = double(typecast(b(25:28), 'single'));
    p.cart_vel  = double(typecast(b(29:32), 'single'));
    p.motor_cmd = double(typecast(b(33:36), 'single'));
    p.odrv_pos  = double(typecast(b(37:40), 'single'));
    p.m7_hb     = typecast(b(41:44), 'uint32');
    p.loop_cnt  = typecast(b(45:48), 'uint32');
    p.overrun   = typecast(b(49:52), 'uint32');
    p.m7_fault  = typecast(b(53:56), 'uint32');
    p.m4_fault  = typecast(b(57:60), 'uint32');
    p.ctrl_mode   = b(61);
    p.cmd_mode    = b(62);
    p.ctrl_src    = b(63);   % 0 = POT, 1 = REMOTE
    p.rail_state  = b(64);   % 0=unhomed 1=ok 2=soft 3=hard 4=latched
    p.cmd_rx      = double(typecast(b(65:68), 'single'));  % datagrams seen on 5005 (was dummy_sine)
    p.command_seq = typecast(b(73:76), 'uint32');   % was dummy_counter, now command_seq
    % Encoder diagnostics block (appended 2026-07-07)
    p.enc_raw     = double(typecast(b(77:82),   'uint16'));  % last raw SPI word
    p.enc_diaagc  = double(typecast(b(83:88),   'uint16'));  % [11]MAGL [10]MAGH [7:0]AGC
    p.enc_par     = double(typecast(b(89:94),   'uint16'));  % parity rejects
    p.enc_ef      = double(typecast(b(95:100),  'uint16'));  % EF rejects
    p.enc_dead    = double(typecast(b(101:106), 'uint16'));  % dead-bus rejects
end

function tt = decode_faults_tooltip(m7, m4)
    % Per-bit elaboration of the fault words, shown on hover over the
    % Faults pill. Names/bits mirror Common/Inc/shared_state.h.
    m7def = { ...
        0, 'LOOP_OVERRUN',   'M7 control loop missed its 200 µs slot (check loop_overrun count)'; ...
        1, 'SPI_TIMEOUT',    'Encoder SPI transaction timed out — SPI peripheral/clock problem'; ...
        2, 'CAN_TX_FAIL',    'FDCAN frame to the ODrive failed to queue — bus off / not wired?'; ...
        3, 'ODRIVE_HB_LOST', 'No ODrive heartbeat on CAN — ODrive off, bus down, or node ID wrong'; ...
        4, 'LIMIT_HIT',      'Rail travel limit tripped — power was cut; see RAIL pill'; ...
        5, 'ENC_DATA',       'Encoder frames rejected (parity / EF / dead bus) — see sensor tiles'; ...
        6, 'ODRIVE_AXIS',    'ODrive axis fault / left closed-loop, or encoder-estimate stream stale, during a run — run auto-aborted'; ...
        7, 'M7_INIT_FAIL',   'M7 hit Error_Handler (init/HAL failure) — M7 dead, heartbeat frozen, CM7 red LED fast-blinking'};
    m4def = { ...
        0, 'WDT_STALE_HB',   'M7 heartbeat went stale — M7 halted/crashed (or debugger held it)'; ...
        1, 'UDP_RX_BADLEN',  'Malformed/out-of-range MATLAB command rejected'; ...
        2, 'UART_BAD_CRC',   'ESP32 UART link frame errors (CRC/length)'; ...
        3, 'HSEM_TIMEOUT',   'Inter-core semaphore timeout'; ...
        4, 'UDP_TX_FAIL',    'Telemetry/ID-log UDP send failed (pbuf alloc / sendto)'};
    tt = {};
    for k = 1:size(m7def,1)
        if bitand(uint32(m7), bitshift(uint32(1), m7def{k,1}))
            tt{end+1} = sprintf('M7 %s — %s', m7def{k,2}, m7def{k,3}); %#ok<AGROW>
        end
    end
    for k = 1:size(m4def,1)
        if bitand(uint32(m4), bitshift(uint32(1), m4def{k,1}))
            tt{end+1} = sprintf('M4 %s — %s', m4def{k,2}, m4def{k,3}); %#ok<AGROW>
        end
    end
    if isempty(tt)
        tt = {'No faults. M7: loop, SPI, CAN, ODrive link, rail, encoders — M4: watchdog, UDP, ESP32 link.'};
    end
end

function p = make_empty_packet()
    p.seq       = uint32(0);
    p.ts        = uint32(0);
    p.angle     = [0.0; 0.0; 0.0];
    p.cart_pos  = 0.0;
    p.cart_vel  = 0.0;
    p.motor_cmd = 0.0;
    p.odrv_pos  = 0.0;
    p.m7_hb     = uint32(0);
    p.loop_cnt  = uint32(0);
    p.overrun   = uint32(0);
    p.m7_fault  = uint32(0);
    p.m4_fault  = uint32(0);
    p.ctrl_mode = uint8(0);
    p.cmd_mode  = uint8(0);
    p.ctrl_src  = uint8(0);
    p.rail_state = uint8(0);
    p.cmd_rx    = 0.0;
    p.command_seq = uint32(0);
    p.enc_raw    = [0 0 0];
    p.enc_diaagc = [0 0 0];
    p.enc_par    = [0 0 0];
    p.enc_ef     = [0 0 0];
    p.enc_dead   = [0 0 0];
end
