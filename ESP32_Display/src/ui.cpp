/**
 * ui.cpp — Professional LVGL kiosk UI for the pendulum demo console.
 *
 *  800x480, dark theme. Three pages under a persistent status bar, switched
 *  by a bottom nav bar: LIVE / CHALLENGE / EXPERT (see ui.h).
 *
 *  The visual language is deliberately restrained: one dark canvas, cards
 *  with a single subtle border, one accent colour, and mode-coded pills.
 *  Everything is styled explicitly here rather than leaning on the LVGL
 *  theme, so it looks the same across LVGL point releases.
 */

#include "ui.h"
#include "uart_link.h"
#include <lvgl.h>
#include <stdio.h>
#include <math.h>

/* ======================================================================
 *  Theme
 * ==================================================================== */
#define COL_BG        lv_color_hex(0x0B0E14)   /* app background      */
#define COL_CARD      lv_color_hex(0x151A24)   /* card surface        */
#define COL_CARD2     lv_color_hex(0x1E2531)   /* raised surface      */
#define COL_BORDER    lv_color_hex(0x2B3444)   /* hairline border     */
#define COL_TXT       lv_color_hex(0xE7EBF3)   /* primary text        */
#define COL_MUTED     lv_color_hex(0x8791A3)   /* secondary text      */
#define COL_ACCENT    lv_color_hex(0x35DB92)   /* brand / OK          */
#define COL_BLUE      lv_color_hex(0x4DA3FF)   /* pot / manual        */
#define COL_AMBER     lv_color_hex(0xF5A623)   /* swing-up / warn     */
#define COL_PURPLE    lv_color_hex(0xA66BFF)   /* system-ID           */
#define COL_RED       lv_color_hex(0xFF4D5E)   /* fault / e-stop      */
#define COL_GREY      lv_color_hex(0x596273)   /* idle                */

#define SCR_W        800
#define SCR_H        480
#define TOPBAR_H      58
#define NAV_H         66
#define CONTENT_Y    (TOPBAR_H)
#define CONTENT_H    (SCR_H - TOPBAR_H - NAV_H)   /* 356 */

/* Challenge thresholds (degrees from upright). */
#define CH_FALL_DEG      30.0f     /* beyond this, the run resets            */
#define UPRIGHT_TOL_DEG  12.0f     /* within this, "upright" indicator green */

/* Cart display mapping: rail-frame position (rev) -> fraction of viz width.
 * CART_DISP_MAX_REV mirrors RAIL_TRAVEL_REV (rail_limits.h) which ships as
 * a 12.0 PLACEHOLDER — update BOTH when the real travel is measured (U4);
 * a mismatch only squeezes the on-screen cart motion (display clamps). */
#define CART_DISP_MIN_REV   0.0f
#define CART_DISP_MAX_REV   12.0f

/* Action codes carried in each button's user_data. */
enum {
    ACT_NONE = 0,
    ACT_MODE_IDLE, ACT_MODE_POTPOS, ACT_MODE_POTVEL,
    ACT_MODE_SWINGUP, ACT_MODE_BALANCE, ACT_ESTOP,
    ACT_HOME, ACT_ZERO, ACT_CLEAR,
    ACT_GAIN_IDX, ACT_GAIN_DEC, ACT_GAIN_INC,
    ACT_CH_MANUAL, ACT_CH_COMPUTER, ACT_CH_STOP,
    ACT_NAV_LIVE, ACT_NAV_CH, ACT_NAV_EXPERT,
};

static const float GAIN_DELTA_STEP = 0.05f;

/* ======================================================================
 *  Reusable pendulum visualisation widget
 * ==================================================================== */
typedef struct {
    lv_obj_t   *card;
    lv_obj_t   *rail;
    lv_obj_t   *cart;
    lv_obj_t   *link[3];
    lv_point_t  rail_pts[2];
    lv_point_t  link_pts[3][2];
    int         w, h;       /* inner canvas size */
    int         rail_y;     /* rail line y within canvas */
    int         pole_len;   /* px per link */
    int         cart_w, cart_h;
} PendViz;

/* ======================================================================
 *  Widget handles
 * ==================================================================== */
static struct {
    /* top bar */
    lv_obj_t *mode_pill,  *mode_lbl;
    lv_obj_t *link_dot,   *link_lbl;
    lv_obj_t *rail_chip,  *rail_lbl;

    /* pages */
    lv_obj_t *page_live, *page_ch, *page_expert;
    lv_obj_t *nav_btn[3];

    /* LIVE */
    PendViz   live_viz;
    lv_obj_t *kpi_val[4];      /* theta, cart pos, cart vel, health */

    /* CHALLENGE */
    PendViz   ch_viz;
    lv_obj_t *ch_timer, *ch_best, *ch_tiltbar, *ch_tilt_lbl, *ch_hint;
    lv_obj_t *ch_overlay;      /* "home first" veil */

    /* EXPERT */
    lv_obj_t *sp_slider, *sp_lbl;
    lv_obj_t *gain_idx_lbl, *gain_val_lbl;
    lv_obj_t *fault_lbl, *stats_lbl, *ack_lbl;
} g;

static int      s_gain_idx = 0;
static float    s_gain_shadow[6] = {0};   /* local echo of accumulated nudges */

/* Challenge scorer + display-zero (captured on Zero-theta). */
static float    s_disp_zero[3]   = {0, 0, 0};
static bool     s_ch_running     = false;
static uint32_t s_ch_start_ms    = 0;
static float    s_ch_best_s      = 0.0f;

/* ======================================================================
 *  Small helpers
 * ==================================================================== */
static float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void no_scroll(lv_obj_t *o)
{
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}

/* A styled card container. */
static lv_obj_t* mk_card(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_size(c, w, h);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_style_bg_color(c, COL_CARD, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(c, COL_BORDER, 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_radius(c, 14, 0);
    lv_obj_set_style_pad_all(c, 12, 0);
    no_scroll(c);
    return c;
}

static lv_obj_t* mk_label(lv_obj_t *parent, const char *txt,
                          const lv_font_t *font, lv_color_t col)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, col, 0);
    return l;
}

/* A pill / chip (rounded filled label holder). Returns the container;
 * *out_lbl gets the inner label. */
static lv_obj_t* mk_pill(lv_obj_t *parent, lv_color_t bg, const lv_font_t *font,
                         lv_obj_t **out_lbl, const char *txt)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_style_bg_color(p, bg, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_hor(p, 16, 0);
    lv_obj_set_style_pad_ver(p, 6, 0);
    no_scroll(p);
    lv_obj_t *l = mk_label(p, txt, font, lv_color_white());
    lv_obj_center(l);
    lv_obj_set_width(p, LV_SIZE_CONTENT);
    lv_obj_set_height(p, LV_SIZE_CONTENT);
    if (out_lbl) *out_lbl = l;
    return p;
}

/* A big touch button with an action code. */
static lv_obj_t* mk_btn(lv_obj_t *parent, const char *txt, int action,
                        lv_color_t bg, const lv_font_t *font,
                        int w, int h, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_color(b, lv_color_darken(bg, LV_OPA_30), LV_STATE_PRESSED);
    lv_obj_set_style_radius(b, 12, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_user_data(b, (void *)(intptr_t)action);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = mk_label(b, txt, font, lv_color_white());
    lv_obj_center(l);
    return b;
}

/* ======================================================================
 *  Mode helpers
 * ==================================================================== */
static const char* mode_name(uint8_t m)
{
    switch (m) {
    case CTRL_MODE_IDLE:         return "IDLE";
    case CTRL_MODE_SWINGUP:      return "SWING-UP";
    case CTRL_MODE_BALANCE:      return "BALANCE";
    case CTRL_MODE_SAFE_STOP:    return "E-STOP";
    case CTRL_MODE_POT_POSITION: return "MANUAL POS";
    case CTRL_MODE_POT_VELOCITY: return "MANUAL VEL";
    case CTRL_MODE_SYSID:        return "SYSTEM-ID";
    default:                     return "----";
    }
}

static lv_color_t mode_color(uint8_t m)
{
    switch (m) {
    case CTRL_MODE_BALANCE:      return COL_ACCENT;
    case CTRL_MODE_SWINGUP:      return COL_AMBER;
    case CTRL_MODE_POT_POSITION:
    case CTRL_MODE_POT_VELOCITY: return COL_BLUE;
    case CTRL_MODE_SYSID:        return COL_PURPLE;
    case CTRL_MODE_SAFE_STOP:    return COL_RED;
    default:                     return COL_GREY;
    }
}

static const char* rail_name(uint8_t r)
{
    switch (r) {
    case RAIL_STATE_UNHOMED:    return "RAIL: UNHOMED";
    case RAIL_STATE_OK:         return "RAIL: OK";
    case RAIL_STATE_SOFT_CLAMP: return "RAIL: LIMIT";
    case RAIL_STATE_HARD_TRIP:  return "RAIL: CUT";
    case RAIL_STATE_LATCHED:    return "RAIL: LATCHED";
    default:                    return "RAIL: ?";
    }
}

static lv_color_t rail_color(uint8_t r)
{
    switch (r) {
    case RAIL_STATE_OK:         return COL_ACCENT;
    case RAIL_STATE_SOFT_CLAMP: return COL_AMBER;
    case RAIL_STATE_HARD_TRIP:
    case RAIL_STATE_LATCHED:    return COL_RED;
    default:                    return COL_GREY;
    }
}

/* ======================================================================
 *  Pendulum visualisation
 * ==================================================================== */
static void pendviz_create(PendViz *pv, lv_obj_t *parent,
                           int x, int y, int w, int h)
{
    pv->card = mk_card(parent, x, y, w, h);
    lv_obj_set_style_bg_color(pv->card, COL_BG, 0);   /* darker "stage" */

    /* Inner canvas geometry (account for 12 px card padding). */
    pv->w = w - 24;
    pv->h = h - 24;
    pv->rail_y   = (int)(pv->h * 0.66f);
    pv->pole_len = (int)(pv->h * 0.30f);
    pv->cart_w   = 74;
    pv->cart_h   = 26;

    /* Rail line. */
    pv->rail = lv_line_create(pv->card);
    pv->rail_pts[0] = (lv_point_t){ (lv_coord_t)(pv->w * 0.06f), (lv_coord_t)pv->rail_y };
    pv->rail_pts[1] = (lv_point_t){ (lv_coord_t)(pv->w * 0.94f), (lv_coord_t)pv->rail_y };
    lv_line_set_points(pv->rail, pv->rail_pts, 2);
    lv_obj_set_style_line_color(pv->rail, COL_BORDER, 0);
    lv_obj_set_style_line_width(pv->rail, 4, 0);
    lv_obj_set_style_line_rounded(pv->rail, true, 0);

    /* Links (drawn base->tip; link 0 is the boldest). */
    for (int i = 0; i < 3; i++) {
        pv->link[i] = lv_line_create(pv->card);
        pv->link_pts[i][0] = (lv_point_t){ 0, 0 };
        pv->link_pts[i][1] = (lv_point_t){ 0, 0 };
        lv_line_set_points(pv->link[i], pv->link_pts[i], 2);
        lv_color_t c = (i == 0) ? COL_ACCENT
                     : (i == 1) ? COL_BLUE : COL_PURPLE;
        lv_obj_set_style_line_color(pv->link[i], c, 0);
        lv_obj_set_style_line_width(pv->link[i], (i == 0) ? 9 : 6, 0);
        lv_obj_set_style_line_rounded(pv->link[i], true, 0);
    }

    /* Cart. */
    pv->cart = lv_obj_create(pv->card);
    lv_obj_set_size(pv->cart, pv->cart_w, pv->cart_h);
    lv_obj_set_style_bg_color(pv->cart, COL_TXT, 0);
    lv_obj_set_style_bg_opa(pv->cart, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(pv->cart, 6, 0);
    lv_obj_set_style_border_width(pv->cart, 0, 0);
    no_scroll(pv->cart);
}

/* angles: raw AS5047P link angles (rad). Only n_links drawn. */
static void pendviz_update(PendViz *pv, const float *angles, int n_links,
                           float cart_pos_rev)
{
    /* Cart x from rail-frame position. */
    float frac = (cart_pos_rev - CART_DISP_MIN_REV) /
                 (CART_DISP_MAX_REV - CART_DISP_MIN_REV);
    if (frac < 0.05f) frac = 0.05f;
    if (frac > 0.95f) frac = 0.95f;
    int pivot_x = (int)(pv->w * (0.06f + frac * 0.88f));
    int pivot_y = pv->rail_y;

    lv_obj_set_pos(pv->cart, pivot_x - pv->cart_w / 2,
                             pivot_y - pv->cart_h / 2);

    /* Chain the links from the pivot. theta = 0 is straight up; +theta
     * tips toward the far (right) end. Screen-y grows downward. */
    int bx = pivot_x, by = pivot_y - pv->cart_h / 2;
    float cum = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (i < n_links) {
            cum += wrap_pi(angles[i] - s_disp_zero[i]);
            int tx = bx + (int)(pv->pole_len * sinf(cum));
            int ty = by - (int)(pv->pole_len * cosf(cum));
            pv->link_pts[i][0] = (lv_point_t){ (lv_coord_t)bx, (lv_coord_t)by };
            pv->link_pts[i][1] = (lv_point_t){ (lv_coord_t)tx, (lv_coord_t)ty };
            lv_line_set_points(pv->link[i], pv->link_pts[i], 2);
            lv_obj_clear_flag(pv->link[i], LV_OBJ_FLAG_HIDDEN);
            bx = tx; by = ty;
        } else {
            lv_obj_add_flag(pv->link[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* ======================================================================
 *  Event handlers
 * ==================================================================== */
static void show_page(int which);   /* fwd */

static void on_action(lv_event_t *e)
{
    int act = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    switch (act) {
    case ACT_MODE_IDLE:    UartLink_SendMode(CTRL_MODE_IDLE);         break;
    case ACT_MODE_POTPOS:  UartLink_SendMode(CTRL_MODE_POT_POSITION); break;
    case ACT_MODE_POTVEL:  UartLink_SendMode(CTRL_MODE_POT_VELOCITY); break;
    case ACT_MODE_SWINGUP: UartLink_SendMode(CTRL_MODE_SWINGUP);      break;
    case ACT_MODE_BALANCE: UartLink_SendMode(CTRL_MODE_BALANCE);      break;
    case ACT_ESTOP:        UartLink_SendMode(CTRL_MODE_SAFE_STOP);    break;

    case ACT_HOME:         UartLink_SendHome();                       break;
    case ACT_ZERO:
        /* Zero the on-screen display too, in the same gesture the M7
         * estimator uses to zero its control angle. */
        UartLink_SendZeroUpright();
        {
            const EspTelemetry_t *t = UartLink_LastTelemetry();
            if (t) for (int i = 0; i < 3; i++) s_disp_zero[i] = t->link_angle_rad[i];
        }
        break;
    case ACT_CLEAR:        UartLink_SendClearErrors();                break;

    case ACT_GAIN_IDX:
        s_gain_idx = (s_gain_idx + 1) % 6;
        lv_label_set_text_fmt(g.gain_idx_lbl, "K%d", s_gain_idx);
        lv_label_set_text_fmt(g.gain_val_lbl, "%+.2f", s_gain_shadow[s_gain_idx]);
        break;
    case ACT_GAIN_DEC:
    case ACT_GAIN_INC: {
        float d = (act == ACT_GAIN_INC) ? GAIN_DELTA_STEP : -GAIN_DELTA_STEP;
        UartLink_SendGainDelta((uint8_t)s_gain_idx, d);
        s_gain_shadow[s_gain_idx] += d;
        if (s_gain_shadow[s_gain_idx] >  50.0f) s_gain_shadow[s_gain_idx] =  50.0f;
        if (s_gain_shadow[s_gain_idx] < -50.0f) s_gain_shadow[s_gain_idx] = -50.0f;
        lv_label_set_text_fmt(g.gain_val_lbl, "%+.2f", s_gain_shadow[s_gain_idx]);
        break;
    }

    case ACT_CH_MANUAL:   UartLink_SendMode(CTRL_MODE_POT_POSITION);  break;
    case ACT_CH_COMPUTER: UartLink_SendMode(CTRL_MODE_BALANCE);       break;
    case ACT_CH_STOP:     UartLink_SendMode(CTRL_MODE_SAFE_STOP);     break;

    case ACT_NAV_LIVE:    show_page(0); break;
    case ACT_NAV_CH:      show_page(1); break;
    case ACT_NAV_EXPERT:  show_page(2); break;
    default: break;
    }
}

static void on_setpoint(lv_event_t *e)
{
    (void)e;
    int v = lv_slider_get_value(g.sp_slider);    /* -1000..1000 */
    float sp = (float)v / 100.0f;                /* -10.00..10.00 */
    lv_label_set_text_fmt(g.sp_lbl, "SETPOINT  %+.2f", sp);
    UartLink_SendSetpoint(sp);
}

/* ======================================================================
 *  Top bar
 * ==================================================================== */
static void build_topbar(lv_obj_t *scr)
{
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_size(bar, SCR_W, TOPBAR_H);
    lv_obj_set_pos(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, COL_CARD, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, COL_BORDER, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    no_scroll(bar);

    lv_obj_t *title = mk_label(bar, "TRIPLE PENDULUM",
                               &lv_font_montserrat_20, COL_TXT);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 18, -8);
    lv_obj_t *sub = mk_label(bar, "control console",
                             &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(sub, LV_ALIGN_LEFT_MID, 20, 12);

    g.mode_pill = mk_pill(bar, COL_GREY, &lv_font_montserrat_20,
                          &g.mode_lbl, "IDLE");
    lv_obj_align(g.mode_pill, LV_ALIGN_CENTER, 0, 0);

    /* rail chip + link dot on the right */
    g.rail_chip = mk_pill(bar, COL_GREY, &lv_font_montserrat_14,
                          &g.rail_lbl, "RAIL: UNHOMED");
    lv_obj_align(g.rail_chip, LV_ALIGN_RIGHT_MID, -120, 0);

    g.link_dot = lv_obj_create(bar);
    lv_obj_set_size(g.link_dot, 14, 14);
    lv_obj_set_style_radius(g.link_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(g.link_dot, 0, 0);
    lv_obj_set_style_bg_color(g.link_dot, COL_RED, 0);
    no_scroll(g.link_dot);
    lv_obj_align(g.link_dot, LV_ALIGN_RIGHT_MID, -92, 0);
    g.link_lbl = mk_label(bar, "LINK", &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(g.link_lbl, LV_ALIGN_RIGHT_MID, -18, 0);
}

/* ======================================================================
 *  Bottom nav
 * ==================================================================== */
static void style_nav(int active)
{
    for (int i = 0; i < 3; i++) {
        bool on = (i == active);
        lv_obj_set_style_bg_color(g.nav_btn[i], on ? COL_CARD2 : COL_CARD, 0);
        lv_obj_t *lbl = lv_obj_get_child(g.nav_btn[i], 0);
        lv_obj_set_style_text_color(lbl, on ? COL_ACCENT : COL_MUTED, 0);
    }
}

static void build_nav(lv_obj_t *scr)
{
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_size(bar, SCR_W, NAV_H);
    lv_obj_set_pos(bar, 0, SCR_H - NAV_H);
    lv_obj_set_style_bg_color(bar, COL_CARD, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_color(bar, COL_BORDER, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    no_scroll(bar);

    const char *names[3] = { "LIVE", "CHALLENGE", "EXPERT" };
    const int   acts[3]  = { ACT_NAV_LIVE, ACT_NAV_CH, ACT_NAV_EXPERT };
    int bw = SCR_W / 3;
    for (int i = 0; i < 3; i++) {
        lv_obj_t *b = lv_btn_create(bar);
        lv_obj_set_size(b, bw - 2, NAV_H - 2);
        lv_obj_set_pos(b, i * bw, 0);
        lv_obj_set_style_bg_color(b, COL_CARD, 0);
        lv_obj_set_style_radius(b, 0, 0);
        lv_obj_set_style_border_width(b, 0, 0);
        lv_obj_set_style_shadow_width(b, 0, 0);
        lv_obj_set_user_data(b, (void *)(intptr_t)acts[i]);
        lv_obj_add_event_cb(b, on_action, LV_EVENT_CLICKED, NULL);
        lv_obj_t *l = mk_label(b, names[i], &lv_font_montserrat_20, COL_MUTED);
        lv_obj_center(l);
        g.nav_btn[i] = b;
    }
}

/* ======================================================================
 *  LIVE page
 * ==================================================================== */
static lv_obj_t* mk_kpi(lv_obj_t *parent, int x, int y, int w, int h,
                        const char *caption, lv_obj_t **out_val)
{
    lv_obj_t *card = mk_card(parent, x, y, w, h);
    lv_obj_t *cap = mk_label(card, caption, &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(cap, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *val = mk_label(card, "--", &lv_font_montserrat_40, COL_TXT);
    lv_obj_align(val, LV_ALIGN_LEFT_MID, 0, 10);
    *out_val = val;
    return card;
}

static void build_live(lv_obj_t *scr)
{
    lv_obj_t *pg = lv_obj_create(scr);
    lv_obj_set_size(pg, SCR_W, CONTENT_H);
    lv_obj_set_pos(pg, 0, CONTENT_Y);
    lv_obj_set_style_bg_color(pg, COL_BG, 0);
    lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pg, 0, 0);
    lv_obj_set_style_radius(pg, 0, 0);
    lv_obj_set_style_pad_all(pg, 12, 0);
    no_scroll(pg);
    g.page_live = pg;

    /* Left: the stage. */
    pendviz_create(&g.live_viz, pg, 0, 0, 452, CONTENT_H - 24);

    /* Right: 2x2 KPI grid. */
    int gx = 464, gw = (SCR_W - 24 - gx - 12) / 2, gh = (CONTENT_H - 24 - 12) / 2;
    mk_kpi(pg, gx,            0,       gw, gh, "TILT FROM UPRIGHT", &g.kpi_val[0]);
    mk_kpi(pg, gx + gw + 12,  0,       gw, gh, "CART POSITION",     &g.kpi_val[1]);
    mk_kpi(pg, gx,            gh + 12, gw, gh, "CART VELOCITY",     &g.kpi_val[2]);
    mk_kpi(pg, gx + gw + 12,  gh + 12, gw, gh, "LOOP HEALTH",       &g.kpi_val[3]);
}

/* ======================================================================
 *  CHALLENGE page
 * ==================================================================== */
static void build_challenge(lv_obj_t *scr)
{
    lv_obj_t *pg = lv_obj_create(scr);
    lv_obj_set_size(pg, SCR_W, CONTENT_H);
    lv_obj_set_pos(pg, 0, CONTENT_Y);
    lv_obj_set_style_bg_color(pg, COL_BG, 0);
    lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pg, 0, 0);
    lv_obj_set_style_radius(pg, 0, 0);
    lv_obj_set_style_pad_all(pg, 12, 0);
    no_scroll(pg);
    g.page_ch = pg;

    /* Left: the pole you're fighting. */
    pendviz_create(&g.ch_viz, pg, 0, 0, 372, CONTENT_H - 24);

    /* Right: the scoreboard. */
    lv_obj_t *card = mk_card(pg, 384, 0, SCR_W - 24 - 384, CONTENT_H - 24);

    lv_obj_t *head = mk_label(card, "CAN YOU BEAT THE COMPUTER?",
                              &lv_font_montserrat_20, COL_TXT);
    lv_obj_align(head, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *cap = mk_label(card, "TIME UPRIGHT", &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 34);

    g.ch_timer = mk_label(card, "0.0", &lv_font_montserrat_48, COL_ACCENT);
    lv_obj_align(g.ch_timer, LV_ALIGN_TOP_MID, 0, 54);

    g.ch_best = mk_label(card, "BEST  0.0 s", &lv_font_montserrat_16, COL_MUTED);
    lv_obj_align(g.ch_best, LV_ALIGN_TOP_MID, 0, 118);

    /* Tilt bar: centred, red near the edges. */
    g.ch_tiltbar = lv_bar_create(card);
    lv_obj_set_size(g.ch_tiltbar, 300, 16);
    lv_obj_align(g.ch_tiltbar, LV_ALIGN_TOP_MID, 0, 156);
    lv_bar_set_range(g.ch_tiltbar, -300, 300);   /* tenths of a degree */
    /* RANGE mode so the indicator is a needle growing from the centre (0)
     * outwards, rather than a meter filling from the left edge. */
    lv_bar_set_mode(g.ch_tiltbar, LV_BAR_MODE_RANGE);
    lv_bar_set_start_value(g.ch_tiltbar, 0, LV_ANIM_OFF);
    lv_bar_set_value(g.ch_tiltbar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g.ch_tiltbar, COL_CARD2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g.ch_tiltbar, COL_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_radius(g.ch_tiltbar, 8, LV_PART_MAIN);

    g.ch_tilt_lbl = mk_label(card, "0.0 deg", &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(g.ch_tilt_lbl, LV_ALIGN_TOP_MID, 0, 178);

    g.ch_hint = mk_label(card,
        "Centre the fader, press TAKE OVER, then slide it to keep the pole up.",
        &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(g.ch_hint, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g.ch_hint, SCR_W - 24 - 384 - 24);
    lv_obj_align(g.ch_hint, LV_ALIGN_TOP_MID, 0, 206);

    /* Action row. */
    int bw = 122, bh = 52, by = CONTENT_H - 24 - 24 - bh;
    lv_obj_t *b1 = mk_btn(card, "TAKE OVER", ACT_CH_MANUAL, COL_BLUE,
                          &lv_font_montserrat_16, bw, bh, on_action);
    lv_obj_align(b1, LV_ALIGN_TOP_LEFT, 0, by);
    lv_obj_t *b2 = mk_btn(card, "LET COMPUTER", ACT_CH_COMPUTER, COL_ACCENT,
                          &lv_font_montserrat_16, bw, bh, on_action);
    lv_obj_align(b2, LV_ALIGN_TOP_MID, 0, by);
    lv_obj_t *b3 = mk_btn(card, "STOP", ACT_CH_STOP, COL_RED,
                          &lv_font_montserrat_16, bw, bh, on_action);
    lv_obj_align(b3, LV_ALIGN_TOP_RIGHT, 0, by);

    /* "Home first" overlay veil (shown while unhomed). */
    g.ch_overlay = lv_obj_create(pg);
    lv_obj_set_size(g.ch_overlay, SCR_W - 24, CONTENT_H - 24);
    lv_obj_align(g.ch_overlay, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(g.ch_overlay, COL_BG, 0);
    lv_obj_set_style_bg_opa(g.ch_overlay, LV_OPA_80, 0);
    lv_obj_set_style_border_width(g.ch_overlay, 0, 0);
    lv_obj_set_style_radius(g.ch_overlay, 14, 0);
    no_scroll(g.ch_overlay);
    lv_obj_t *ov = mk_label(g.ch_overlay, "HOME THE RAIL TO BEGIN",
                            &lv_font_montserrat_28, COL_TXT);
    lv_obj_align(ov, LV_ALIGN_CENTER, 0, -16);
    lv_obj_t *ovb = mk_btn(g.ch_overlay, "HOME NOW", ACT_HOME, COL_ACCENT,
                           &lv_font_montserrat_20, 200, 56, on_action);
    lv_obj_align(ovb, LV_ALIGN_CENTER, 0, 52);
}

/* ======================================================================
 *  EXPERT page
 * ==================================================================== */
static void build_expert(lv_obj_t *scr)
{
    lv_obj_t *pg = lv_obj_create(scr);
    lv_obj_set_size(pg, SCR_W, CONTENT_H);
    lv_obj_set_pos(pg, 0, CONTENT_Y);
    lv_obj_set_style_bg_color(pg, COL_BG, 0);
    lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pg, 0, 0);
    lv_obj_set_style_radius(pg, 0, 0);
    lv_obj_set_style_pad_all(pg, 12, 0);
    no_scroll(pg);
    g.page_expert = pg;

    /* --- Left card: modes + actions --- */
    lv_obj_t *left = mk_card(pg, 0, 0, 384, CONTENT_H - 24);
    lv_obj_t *lc = mk_label(left, "MODE", &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(lc, LV_ALIGN_TOP_LEFT, 0, 0);

    struct { const char *t; int a; lv_color_t c; } modes[6] = {
        { "IDLE",     ACT_MODE_IDLE,    COL_GREY  },
        { "MANUAL P", ACT_MODE_POTPOS,  COL_BLUE  },
        { "MANUAL V", ACT_MODE_POTVEL,  COL_BLUE  },
        { "SWING-UP", ACT_MODE_SWINGUP, COL_AMBER },
        { "BALANCE",  ACT_MODE_BALANCE, COL_ACCENT},
        { "E-STOP",   ACT_ESTOP,        COL_RED   },
    };
    int mw = 168, mh = 46;
    for (int i = 0; i < 6; i++) {
        lv_obj_t *b = mk_btn(left, modes[i].t, modes[i].a, modes[i].c,
                             &lv_font_montserrat_16, mw, mh, on_action);
        lv_obj_align(b, LV_ALIGN_TOP_LEFT, (i % 2) * (mw + 12),
                     24 + (i / 2) * (mh + 10));
    }

    struct { const char *t; int a; } acts[3] = {
        { "HOME", ACT_HOME }, { "ZERO \xCE\xB8", ACT_ZERO }, { "CLEAR", ACT_CLEAR },
    };
    int aw = 108, ah = 46, ay = 24 + 3 * (mh + 10) + 8;
    for (int i = 0; i < 3; i++) {
        lv_obj_t *b = mk_btn(left, acts[i].t, acts[i].a, COL_CARD2,
                             &lv_font_montserrat_16, aw, ah, on_action);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_border_color(b, COL_BORDER, 0);
        lv_obj_align(b, LV_ALIGN_TOP_LEFT, i * (aw + 12), ay);
    }

    /* --- Right card: setpoint + gains + faults --- */
    lv_obj_t *right = mk_card(pg, 396, 0, SCR_W - 24 - 396, CONTENT_H - 24);

    g.sp_lbl = mk_label(right, "SETPOINT  +0.00", &lv_font_montserrat_16, COL_TXT);
    lv_obj_align(g.sp_lbl, LV_ALIGN_TOP_LEFT, 0, 0);
    g.sp_slider = lv_slider_create(right);
    lv_obj_set_size(g.sp_slider, SCR_W - 24 - 396 - 24, 16);
    lv_obj_align(g.sp_slider, LV_ALIGN_TOP_LEFT, 0, 28);
    lv_slider_set_range(g.sp_slider, -1000, 1000);
    lv_slider_set_value(g.sp_slider, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g.sp_slider, COL_CARD2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(g.sp_slider, COL_BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(g.sp_slider, COL_BLUE, LV_PART_KNOB);
    lv_obj_add_event_cb(g.sp_slider, on_setpoint, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *gc = mk_label(right, "LIVE LQR GAIN NUDGE",
                            &lv_font_montserrat_14, COL_MUTED);
    lv_obj_align(gc, LV_ALIGN_TOP_LEFT, 0, 62);

    lv_obj_t *bidx = mk_btn(right, "K0", ACT_GAIN_IDX, COL_CARD2,
                            &lv_font_montserrat_20, 66, 46, on_action);
    lv_obj_set_style_border_width(bidx, 1, 0);
    lv_obj_set_style_border_color(bidx, COL_BORDER, 0);
    lv_obj_align(bidx, LV_ALIGN_TOP_LEFT, 0, 86);
    g.gain_idx_lbl = lv_obj_get_child(bidx, 0);

    lv_obj_t *bdec = mk_btn(right, "-", ACT_GAIN_DEC, COL_CARD2,
                            &lv_font_montserrat_28, 60, 46, on_action);
    lv_obj_set_style_border_width(bdec, 1, 0);
    lv_obj_set_style_border_color(bdec, COL_BORDER, 0);
    lv_obj_align(bdec, LV_ALIGN_TOP_LEFT, 78, 86);

    g.gain_val_lbl = mk_label(right, "+0.00", &lv_font_montserrat_28, COL_TXT);
    lv_obj_align(g.gain_val_lbl, LV_ALIGN_TOP_LEFT, 152, 94);

    lv_obj_t *binc = mk_btn(right, "+", ACT_GAIN_INC, COL_CARD2,
                            &lv_font_montserrat_28, 60, 46, on_action);
    lv_obj_set_style_border_width(binc, 1, 0);
    lv_obj_set_style_border_color(binc, COL_BORDER, 0);
    lv_obj_align(binc, LV_ALIGN_TOP_RIGHT, 0, 86);

    g.fault_lbl = mk_label(right, "FAULTS: none", &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(g.fault_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g.fault_lbl, SCR_W - 24 - 396 - 24);
    lv_obj_align(g.fault_lbl, LV_ALIGN_TOP_LEFT, 0, 150);

    g.stats_lbl = mk_label(right, "loop --   overrun --",
                           &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(g.stats_lbl, LV_ALIGN_BOTTOM_LEFT, 0, -22);
    g.ack_lbl = mk_label(right, "ack --", &lv_font_montserrat_12, COL_MUTED);
    lv_obj_align(g.ack_lbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
}

/* ======================================================================
 *  Page switching
 * ==================================================================== */
static void show_page(int which)
{
    lv_obj_add_flag(g.page_live,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g.page_ch,     LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g.page_expert, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *p = (which == 0) ? g.page_live
                : (which == 1) ? g.page_ch : g.page_expert;
    lv_obj_clear_flag(p, LV_OBJ_FLAG_HIDDEN);
    style_nav(which);

    /* Fresh stopwatch each time the challenge is opened (best time is kept). */
    if (which == 1) s_ch_running = false;
}

/* ======================================================================
 *  Public: init + updates
 * ==================================================================== */
void Ui_Init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(scr, COL_TXT, 0);
    no_scroll(scr);

    build_topbar(scr);
    build_live(scr);
    build_challenge(scr);
    build_expert(scr);
    build_nav(scr);

    show_page(0);
}

static void update_challenge(const EspTelemetry_t *t, uint32_t now_ms)
{
    /* Display-zeroed link-1 tilt. */
    float th = wrap_pi(t->link_angle_rad[0] - s_disp_zero[0]);
    float tilt_deg = fabsf(th) * 57.29578f;
    float signed_deg = th * 57.29578f;

    /* Scorer. */
    bool up = (tilt_deg < CH_FALL_DEG);
    if (up) {
        if (!s_ch_running) { s_ch_running = true; s_ch_start_ms = now_ms; }
        float cur = (float)(now_ms - s_ch_start_ms) / 1000.0f;
        if (cur > s_ch_best_s) s_ch_best_s = cur;
        lv_label_set_text_fmt(g.ch_timer, "%.1f", cur);
        lv_obj_set_style_text_color(g.ch_timer,
            (tilt_deg < UPRIGHT_TOL_DEG) ? COL_ACCENT : COL_AMBER, 0);
    } else {
        s_ch_running = false;
        lv_label_set_text(g.ch_timer, "0.0");
        lv_obj_set_style_text_color(g.ch_timer, COL_RED, 0);
    }
    lv_label_set_text_fmt(g.ch_best, "BEST  %.1f s", s_ch_best_s);

    int bar = (int)(signed_deg * 10.0f);
    if (bar > 300) bar = 300;
    if (bar < -300) bar = -300;
    /* Needle from centre: RANGE start<=value, so order by sign. */
    if (bar >= 0) {
        lv_bar_set_start_value(g.ch_tiltbar, 0, LV_ANIM_OFF);
        lv_bar_set_value(g.ch_tiltbar, bar, LV_ANIM_OFF);
    } else {
        lv_bar_set_start_value(g.ch_tiltbar, bar, LV_ANIM_OFF);
        lv_bar_set_value(g.ch_tiltbar, 0, LV_ANIM_OFF);
    }
    lv_obj_set_style_bg_color(g.ch_tiltbar,
        (tilt_deg < UPRIGHT_TOL_DEG) ? COL_ACCENT
      : (tilt_deg < CH_FALL_DEG)     ? COL_AMBER : COL_RED, LV_PART_INDICATOR);
    lv_label_set_text_fmt(g.ch_tilt_lbl, "%+.1f deg", signed_deg);

    pendviz_update(&g.ch_viz, (const float *)t->link_angle_rad, 1, t->cart_position);

    /* Overlay only while unhomed. */
    if (t->rail_state == RAIL_STATE_UNHOMED)
        lv_obj_clear_flag(g.ch_overlay, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(g.ch_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void decode_faults(char *buf, size_t n, uint32_t m7, uint32_t m4)
{
    if (!m7 && !m4) { snprintf(buf, n, "FAULTS: none"); return; }
    int k = snprintf(buf, n, "FAULTS:");
    struct { uint32_t bit; const char *nm; } f7[] = {
        { M7_FAULT_LOOP_OVERRUN, " overrun" }, { M7_FAULT_SPI_TIMEOUT, " spi" },
        { M7_FAULT_CAN_TX_FAIL, " can" }, { M7_FAULT_ODRIVE_HB_LOST, " odrv" },
        { M7_FAULT_LIMIT_HIT, " limit" }, { M7_FAULT_ENC_DATA, " enc" },
        { M7_FAULT_ODRIVE_AXIS, " axis" }, { M7_FAULT_INIT_FAIL, " m7dead" },
    };
    for (unsigned i = 0; i < 8 && k < (int)n; i++)
        if (m7 & f7[i].bit) k += snprintf(buf + k, n - k, "%s", f7[i].nm);
    struct { uint32_t bit; const char *nm; } f4[] = {
        { M4_FAULT_WDT_STALE_HB, " hb" }, { M4_FAULT_UDP_RX_BADLEN, " udprx" },
        { M4_FAULT_UART_BAD_CRC, " uart" }, { M4_FAULT_HSEM_TIMEOUT, " hsem" },
        { M4_FAULT_UDP_TX_FAIL, " udptx" },
    };
    for (unsigned i = 0; i < 5 && k < (int)n; i++)
        if (m4 & f4[i].bit) k += snprintf(buf + k, n - k, "%s", f4[i].nm);
}

void Ui_UpdateTelemetry(const EspTelemetry_t *t, uint32_t now_ms)
{
    /* ---- Link status dot ---- */
    if (t == NULL) {
        lv_obj_set_style_bg_color(g.link_dot, COL_RED, 0);
        lv_label_set_text(g.link_lbl, "NO LINK");
        lv_obj_set_style_text_color(g.link_lbl, COL_RED, 0);
        lv_label_set_text(g.mode_lbl, "----");
        lv_obj_set_style_bg_color(g.mode_pill, COL_GREY, 0);
        return;
    }
    lv_obj_set_style_bg_color(g.link_dot, COL_ACCENT, 0);
    lv_label_set_text(g.link_lbl, "LINK");
    lv_obj_set_style_text_color(g.link_lbl, COL_MUTED, 0);

    /* ---- Mode pill ---- */
    lv_label_set_text(g.mode_lbl, mode_name(t->controller_mode));
    lv_obj_set_style_bg_color(g.mode_pill, mode_color(t->controller_mode), 0);

    /* ---- Rail chip ---- */
    lv_label_set_text(g.rail_lbl, rail_name(t->rail_state));
    lv_obj_set_style_bg_color(g.rail_chip, rail_color(t->rail_state), 0);

    /* ---- LIVE page ---- */
    if (!lv_obj_has_flag(g.page_live, LV_OBJ_FLAG_HIDDEN)) {
        float th = wrap_pi(t->link_angle_rad[0] - s_disp_zero[0]) * 57.29578f;
        lv_label_set_text_fmt(g.kpi_val[0], "%+.1f\xC2\xB0", th);
        lv_obj_set_style_text_color(g.kpi_val[0],
            (fabsf(th) < UPRIGHT_TOL_DEG) ? COL_ACCENT : COL_TXT, 0);
        lv_label_set_text_fmt(g.kpi_val[1], "%+.2f", t->cart_position);
        lv_label_set_text_fmt(g.kpi_val[2], "%+.2f", t->cart_velocity);
        bool healthy = (t->m7_fault_flags == 0) && (t->loop_overrun_cnt == 0);
        lv_label_set_text(g.kpi_val[3], healthy ? "OK" : "CHK");
        lv_obj_set_style_text_color(g.kpi_val[3], healthy ? COL_ACCENT : COL_AMBER, 0);
        pendviz_update(&g.live_viz, (const float *)t->link_angle_rad, 1,
                       t->cart_position);
    }

    /* ---- CHALLENGE page (scored only while visible; best time persists
     * across visits — the stopwatch restarts on page entry). NOTE: BEST is
     * mode-agnostic, so "LET COMPUTER" can set it too — accepted for now;
     * gate on controller_mode == POT_POSITION if human-only scoring is
     * wanted for the demo. ---- */
    if (!lv_obj_has_flag(g.page_ch, LV_OBJ_FLAG_HIDDEN))
        update_challenge(t, now_ms);

    /* ---- EXPERT page ---- */
    if (!lv_obj_has_flag(g.page_expert, LV_OBJ_FLAG_HIDDEN)) {
        char fb[96]; decode_faults(fb, sizeof(fb), t->m7_fault_flags, t->m4_fault_flags);
        lv_label_set_text(g.fault_lbl, fb);
        lv_obj_set_style_text_color(g.fault_lbl,
            (t->m7_fault_flags | t->m4_fault_flags) ? COL_RED : COL_MUTED, 0);
        lv_label_set_text_fmt(g.stats_lbl, "loop %lu   overrun %lu",
            (unsigned long)t->loop_count, (unsigned long)t->loop_overrun_cnt);
    }
}

void Ui_SetAckStatus(uint8_t status, uint32_t seq)
{
    static const char *names[] = { "OK", "BAD_CRC", "BAD_TYPE", "BAD_LEN", "RANGE" };
    const char *n = (status < 5) ? names[status] : "?";
    if (g.ack_lbl) {
        lv_label_set_text_fmt(g.ack_lbl, "ack #%lu  %s", (unsigned long)seq, n);
        lv_obj_set_style_text_color(g.ack_lbl,
            (status == ESPLINK_ACK_OK) ? COL_ACCENT : COL_AMBER, 0);
    }
}
