#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "src/touch/esp_lcd_touch_gt911.h"
#include "src/lcd/esp_lcd_jd9165.h"
#include "lvgl_port_v9.h"
#include "demos/lv_demos.h"
#include "driver/ppa.h"

#define TAG                                 "main"


#define BSP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

#define BSP_LCD_DPI_BUFFER_NUMS             (1)


#define BSP_I2C_NUM                         (I2C_NUM_1)
#define BSP_I2C_SDA                         (GPIO_NUM_7)
#define BSP_I2C_SCL                         (GPIO_NUM_8)

#define BSP_LCD_TOUCH_RST                   (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_INT                   (GPIO_NUM_NC)

#define BSP_LCD_RST                         (GPIO_NUM_5)

i2c_master_bus_handle_t i2c_handle = NULL; 


#define BSP_LCD_BACKLIGHT   GPIO_NUM_23
#define LCD_LEDC_CH         LEDC_CHANNEL_0
static esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
    return ESP_OK;
}

static esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
    return ESP_OK;
}

static esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

static esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

IRAM_ATTR static bool mipi_dsi_lcd_on_vsync_event(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    return lvgl_port_notify_lcd_vsync();
}

/* ===== TU FUNCION GRAFICA ===== */

/* =======================================================================
 * Estado compartido entre callbacks — static para persistir entre calls
 * ======================================================================= */
typedef struct {
    lv_obj_t          *chart;
    lv_chart_series_t *ser_red;
    lv_chart_series_t *ser_green;
    lv_chart_series_t *ser_blue;
    lv_obj_t          *status_lbl;
    lv_timer_t        *acq_timer;
    bool               running;       /* Start/Stop toggle         */
    bool               dark_saved;    /* Dark capturado            */
    bool               ref_saved;     /* Reference capturado       */
    int                spec_mode;     /* 0=Normal 1=Dark 2=Ref     */
} spectro_ctx_t;

/* -------------------------------------------------------------------
 * Macro Gaussiana entera (sin float)
 * ------------------------------------------------------------------- */
#define GAUSS(x, mu, sig, amp) ({                               \
    int _d = (x) - (mu);                                        \
    int32_t _r = (int32_t)_d * _d * 1000 / (2*(sig)*(sig));    \
    int32_t _b = 1000 - _r * 1000 / 6000;                      \
    (int16_t)((_b > 0) ? (int32_t)(amp) * _b * _b / 1000000 : 0); \
})

/* -------------------------------------------------------------------
 * Rellena la grafica con espectro simulado (Gaussianos)
 * mode: 0=normal  1=dark(ruido bajo)  2=reference(plano)
 * ------------------------------------------------------------------- */
static void fill_spectrum(spectro_ctx_t *ctx, int mode)
{
    lv_chart_set_point_count(ctx->chart, 200);

    for(int i = 0; i < 200; i++)
    {
        int nm = 380 + i * 2;
        int16_t r, g, b;

        if(mode == 1) /* Dark: solo ruido termico bajo */
        {
            r = 5  + (lv_rand(0, 10));
            g = 5  + (lv_rand(0, 10));
            b = 5  + (lv_rand(0, 10));
        }
        else if(mode == 2) /* Reference: espectro plano */
        {
            r = 500; g = 500; b = 500;
        }
        else /* Normal: picos de emision reales */
        {
            r = 20;
            r += GAUSS(nm, 589,  8, 820);
            r += GAUSS(nm, 656, 10, 540);
            r += GAUSS(nm, 706,  9, 310);
            if(r > 1000) r = 1000;

            g = 15;
            g += GAUSS(nm, 532,  7, 950);
            g += GAUSS(nm, 546,  8, 280);
            g += GAUSS(nm, 436,  7, 220);
            if(g > 1000) g = 1000;

            b = 10;
            b += GAUSS(nm, 405,  9, 580);
            b += GAUSS(nm, 447,  8, 210);
            b += GAUSS(nm, 436,  6, 290);
            if(b > 1000) b = 1000;
        }

        lv_chart_set_next_value(ctx->chart, ctx->ser_red,   r);
        lv_chart_set_next_value(ctx->chart, ctx->ser_green, g);
        lv_chart_set_next_value(ctx->chart, ctx->ser_blue,  b);
    }

    lv_chart_refresh(ctx->chart);
}

/* -------------------------------------------------------------------
 * Timer: adquisicion continua (simula ruido sobre los picos)
 * ------------------------------------------------------------------- */
static void acq_timer_cb(lv_timer_t *t)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_timer_get_user_data(t);

    /* Agrega un punto nuevo con pequeno ruido sobre el espectro base */
    int nm_last = 778; /* ultimo nm del barrido — solo para variacion */
    int16_t r = 20 + GAUSS(nm_last, 656, 10, 540) + (int16_t)(lv_rand(0,30) - 15);
    int16_t g = 15 + GAUSS(nm_last, 532,  7, 950) + (int16_t)(lv_rand(0,30) - 15);
    int16_t b = 10 + GAUSS(nm_last, 405,  9, 580) + (int16_t)(lv_rand(0,30) - 15);
    if(r < 0) r = 0; if(r > 1000) r = 1000;
    if(g < 0) g = 0; if(g > 1000) g = 1000;
    if(b < 0) b = 0; if(b > 1000) b = 1000;

    lv_chart_set_next_value(ctx->chart, ctx->ser_red,   r);
    lv_chart_set_next_value(ctx->chart, ctx->ser_green, g);
    lv_chart_set_next_value(ctx->chart, ctx->ser_blue,  b);
    lv_chart_refresh(ctx->chart);
}

/* -------------------------------------------------------------------
 * Callback: boton Start/Stop
 * ------------------------------------------------------------------- */
static void btn_start_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_event_get_user_data(e);
    lv_obj_t *btn      = lv_event_get_target(e);
    lv_obj_t *lbl      = lv_obj_get_child(btn, 0);

    ctx->running = !ctx->running;

    if(ctx->running)
    {
        lv_label_set_text(lbl, "Stop");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xE24B4A), 0); /* rojo = activo */
        lv_timer_resume(ctx->acq_timer);
        lv_label_set_text(ctx->status_lbl,
            "ADQUIRIENDO   Integration: 100ms   Avg: 16   Range: 380-780 nm");
    }
    else
    {
        lv_label_set_text(lbl, "Start");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1C2330), 0); /* normal */
        lv_timer_pause(ctx->acq_timer);
        lv_label_set_text(ctx->status_lbl,
            "PAUSADO   Integration: 100ms   Avg: 16   Range: 380-780 nm");
    }
}

/* -------------------------------------------------------------------
 * Callback: boton Single — captura un solo barrido completo
 * ------------------------------------------------------------------- */
static void btn_single_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_event_get_user_data(e);

    /* Detiene adquisicion continua si estaba corriendo */
    if(ctx->running)
    {
        ctx->running = false;
        lv_timer_pause(ctx->acq_timer);
    }

    fill_spectrum(ctx, 0);
    lv_label_set_text(ctx->status_lbl,
        "SINGLE CAPTURE OK   Integration: 100ms   Avg: 1   Range: 380-780 nm");
}

/* -------------------------------------------------------------------
 * Callback: boton Dark — guarda espectro de fondo
 * ------------------------------------------------------------------- */
static void btn_dark_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_event_get_user_data(e);
    lv_obj_t *btn      = lv_event_get_target(e);

    fill_spectrum(ctx, 1); /* muestra dark en grafica */
    ctx->dark_saved = true;
    ctx->spec_mode  = 1;

    lv_obj_set_style_bg_color(btn, lv_color_hex(0x534AB7), 0); /* acento purple */
    lv_label_set_text(ctx->status_lbl,
        "DARK GUARDADO   Ruido termico capturado   Range: 380-780 nm");
}

/* -------------------------------------------------------------------
 * Callback: boton Reference
 * ------------------------------------------------------------------- */
static void btn_ref_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_event_get_user_data(e);
    lv_obj_t *btn      = lv_event_get_target(e);

    fill_spectrum(ctx, 2); /* muestra referencia plana */
    ctx->ref_saved = true;
    ctx->spec_mode = 2;

    lv_obj_set_style_bg_color(btn, lv_color_hex(0x534AB7), 0);
    lv_label_set_text(ctx->status_lbl,
        "REFERENCE GUARDADO   Espectro de referencia capturado   Range: 380-780 nm");
}

/* -------------------------------------------------------------------
 * Callback: boton Options — cicla integracion: 50/100/200/500 ms
 * ------------------------------------------------------------------- */
static void btn_options_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx   = (spectro_ctx_t *)lv_event_get_user_data(e);
    static int int_idx   = 1; /* indice en la tabla */
    static const int int_times[] = { 50, 100, 200, 500 };
    static const uint32_t timer_periods[] = { 400, 800, 1600, 4000 };

    int_idx = (int_idx + 1) % 4;
    lv_timer_set_period(ctx->acq_timer, timer_periods[int_idx]);

    char buf[96];
    lv_snprintf(buf, sizeof(buf),
        "Device: OES-Sim v2.1   Integration: %dms   Avg: 16   Range: 380-780 nm",
        int_times[int_idx]);
    lv_label_set_text(ctx->status_lbl, buf);
}

/* -------------------------------------------------------------------
 * Callback: boton Spectrum — alterna Normal / Dark / Reference
 * ------------------------------------------------------------------- */
static void btn_spectrum_cb(lv_event_t *e)
{
    spectro_ctx_t *ctx = (spectro_ctx_t *)lv_event_get_user_data(e);

    ctx->spec_mode = (ctx->spec_mode + 1) % 3;
    fill_spectrum(ctx, ctx->spec_mode);

    const char *mode_names[] = { "NORMAL", "DARK", "REFERENCE" };
    char buf[80];
    lv_snprintf(buf, sizeof(buf),
        "Vista: %s   Integration: 100ms   Range: 380-780 nm",
        mode_names[ctx->spec_mode]);
    lv_label_set_text(ctx->status_lbl, buf);
}

/* -------------------------------------------------------------------
 * Callback: checkboxes de serie (oculta/muestra la linea)
 * ------------------------------------------------------------------- */
static void cb_series_cb(lv_event_t *e)
{
    lv_obj_t      *cb  = lv_event_get_target(e);
    lv_obj_t      *chart = (lv_obj_t *)lv_event_get_user_data(e);
    bool           checked = lv_obj_has_state(cb, LV_STATE_CHECKED);

    /* Identificamos cual serie es por el indice del checkbox en su padre */
    /* cb1=idx0 rojo  cb2=idx1 verde  cb3=idx2 azul                      */
    lv_obj_t *parent = lv_obj_get_parent(cb);
    uint32_t idx = 0;
    uint32_t cnt = lv_obj_get_child_count(parent);
    for(uint32_t i = 0; i < cnt; i++)
    {
        if(lv_obj_get_child(parent, i) == cb) { idx = i; break; }
    }

    /* Los checkboxes son los hijos 2, 3, 4 del panel (0=title, 1=peaks_title...) */
    /* Usamos el color de la serie para identificarla                              */
    lv_chart_series_t *ser = lv_chart_get_series_next(chart, NULL);
    uint32_t ser_idx = 0;
    /* cb alineados: cb1 primer checkbox=serie 0 (red), cb2=1(green), cb3=2(blue) */
    /* Buscamos el checkbox por posicion relativa entre checkboxes del panel       */
    uint32_t cb_order = 0;
    for(uint32_t i = 0; i < cnt; i++)
    {
        lv_obj_t *child = lv_obj_get_child(parent, i);
        if(child == cb) break;
        if(lv_obj_check_type(child, &lv_checkbox_class)) cb_order++;
    }

    ser = lv_chart_get_series_next(chart, NULL);
    for(uint32_t j = 0; j < cb_order && ser != NULL; j++)
        ser = lv_chart_get_series_next(chart, ser);

    if(ser == NULL) return;

    if(checked)
        lv_chart_hide_series(chart, ser, false);  // mostrar
    else
        lv_chart_hide_series(chart, ser, true);   // ocultar

    lv_chart_refresh(chart);
}

/* ===================================================================
 * FUNCION PRINCIPAL
 * =================================================================== */
void create_graph_ui(void)
{
    /* Contexto estatico — persiste entre callbacks */
    static spectro_ctx_t ctx = { 0 };
    ctx.running   = false;
    ctx.dark_saved= false;
    ctx.ref_saved = false;
    ctx.spec_mode = 0;

    lv_obj_t *scr = lv_screen_active();

    /* ===== Fondo ===== */
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0D1117), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* ===== Barra superior ===== */
    lv_obj_t *top = lv_obj_create(scr);
    lv_obj_set_size(top, 1024, 70);
    lv_obj_align(top, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_radius(top, 0, 0);
    lv_obj_set_style_bg_color(top, lv_color_hex(0x161B22), 0);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(top, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(top, lv_color_hex(0x1D9E75), 0);
    lv_obj_set_style_border_width(top, 1, 0);
    lv_obj_set_style_pad_all(top, 0, 0);
    lv_obj_remove_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *logo = lv_label_create(top);
    lv_label_set_text(logo, "OES-7  OPTICAL EMISSION SPECTROMETER");
    lv_obj_align(logo, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_set_style_text_color(logo, lv_color_hex(0xE6EDF3), 0);
    lv_obj_set_style_text_font(logo, &lv_font_montserrat_14, 0);

    /* Botones ribbon con callbacks */
    typedef void (*btn_cb_t)(lv_event_t *);
    const char  *names[]    = { "Start", "Single", "Dark", "Reference", "Options", "Spectrum" };
    btn_cb_t     callbacks[]= { btn_start_cb, btn_single_cb, btn_dark_cb,
                                btn_ref_cb,   btn_options_cb, btn_spectrum_cb };

    for(int i = 0; i < 6; i++)
    {
        lv_obj_t *btn = lv_button_create(top);
        lv_obj_set_size(btn, 90, 50);
        lv_obj_align(btn, LV_ALIGN_RIGHT_MID, -10 - (5 - i) * 98, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1C2330), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x30363D), 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_radius(btn, 4, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1D9E75), LV_STATE_PRESSED);
        lv_obj_add_event_cb(btn, callbacks[i], LV_EVENT_CLICKED, &ctx);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, names[i]);
        lv_obj_center(lbl);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xE6EDF3), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    }

    /* ===== Panel izquierdo ===== */
    lv_obj_t *side = lv_obj_create(scr);
    lv_obj_set_size(side, 220, 490);
    lv_obj_align(side, LV_ALIGN_TOP_LEFT, 0, 71);
    lv_obj_set_style_bg_color(side, lv_color_hex(0x161B22), 0);
    lv_obj_set_style_bg_opa(side, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(side, lv_color_hex(0x30363D), 0);
    lv_obj_set_style_border_width(side, 1, 0);
    lv_obj_set_style_radius(side, 0, 0);
    lv_obj_set_style_pad_all(side, 0, 0);
    lv_obj_remove_flag(side, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(side);
    lv_label_set_text(title, "Spectral Data");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_text_color(title, lv_color_hex(0xE6EDF3), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);

    /* Checkboxes de serie — user_data = chart (se asigna despues) */
    lv_obj_t *cb1 = lv_checkbox_create(side);
    lv_checkbox_set_text(cb1, "Serie Roja");
    lv_obj_align(cb1, LV_ALIGN_TOP_LEFT, 10, 50);
    lv_obj_add_state(cb1, LV_STATE_CHECKED);
    lv_obj_set_style_text_color(cb1, lv_color_hex(0x8B949E), 0);
    lv_obj_set_style_bg_color(cb1, lv_color_hex(0xE24B4A), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(cb1, lv_color_hex(0xE24B4A), LV_PART_INDICATOR);

    lv_obj_t *cb2 = lv_checkbox_create(side);
    lv_checkbox_set_text(cb2, "Serie Verde");
    lv_obj_align(cb2, LV_ALIGN_TOP_LEFT, 10, 85);
    lv_obj_add_state(cb2, LV_STATE_CHECKED);
    lv_obj_set_style_text_color(cb2, lv_color_hex(0x8B949E), 0);
    lv_obj_set_style_bg_color(cb2, lv_color_hex(0x1D9E75), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(cb2, lv_color_hex(0x1D9E75), LV_PART_INDICATOR);

    lv_obj_t *cb3 = lv_checkbox_create(side);
    lv_checkbox_set_text(cb3, "Serie Azul");
    lv_obj_align(cb3, LV_ALIGN_TOP_LEFT, 10, 120);
    lv_obj_add_state(cb3, LV_STATE_CHECKED);
    lv_obj_set_style_text_color(cb3, lv_color_hex(0x8B949E), 0);
    lv_obj_set_style_bg_color(cb3, lv_color_hex(0x378ADD), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(cb3, lv_color_hex(0x378ADD), LV_PART_INDICATOR);

    /* Picos identificados */
    lv_obj_t *peaks_title = lv_label_create(side);
    lv_label_set_text(peaks_title, "Picos Identificados");
    lv_obj_align(peaks_title, LV_ALIGN_TOP_LEFT, 10, 165);
    lv_obj_set_style_text_color(peaks_title, lv_color_hex(0x8B949E), 0);
    lv_obj_set_style_text_font(peaks_title, &lv_font_montserrat_14, 0);

    const char   *peak_text[]  = { "404.6 nm  Hg I", "532.0 nm  Nd:YAG",
                                   "589.0 nm  Na I",  "656.3 nm  H-alfa" };
    lv_color_t    peak_color[] = { lv_color_hex(0x8B63D4), lv_color_hex(0x1D9E75),
                                   lv_color_hex(0xEF9F27), lv_color_hex(0xE24B4A) };
    for(int i = 0; i < 4; i++)
    {
        lv_obj_t *dot = lv_obj_create(side);
        lv_obj_set_size(dot, 6, 6);
        lv_obj_align(dot, LV_ALIGN_TOP_LEFT, 12, 185 + i * 20);
        lv_obj_set_style_bg_color(dot, peak_color[i], 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *pl = lv_label_create(side);
        lv_label_set_text(pl, peak_text[i]);
        lv_obj_align(pl, LV_ALIGN_TOP_LEFT, 24, 182 + i * 20);
        lv_obj_set_style_text_color(pl, lv_color_hex(0x8B949E), 0);
        lv_obj_set_style_text_font(pl, &lv_font_montserrat_14, 0);
    }

    /* ===== Grafica ===== */
    lv_obj_t *chart = lv_chart_create(scr);
    lv_obj_set_size(chart, 792, 490);
    lv_obj_align(chart, LV_ALIGN_TOP_RIGHT, 0, 71);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_point_count(chart, 200);
    lv_chart_set_div_line_count(chart, 5, 10);
    lv_obj_set_style_bg_color(chart, lv_color_hex(0x0D1117), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x30363D), 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_line_color(chart, lv_color_hex(0x30363D), LV_PART_MAIN);
    lv_obj_set_style_line_opa(chart, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_radius(chart, 0, 0);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);

    ctx.chart     = chart;
    ctx.ser_red   = lv_chart_add_series(chart, lv_color_hex(0xE24B4A), LV_CHART_AXIS_PRIMARY_Y);
    ctx.ser_green = lv_chart_add_series(chart, lv_color_hex(0x1D9E75), LV_CHART_AXIS_PRIMARY_Y);
    ctx.ser_blue  = lv_chart_add_series(chart, lv_color_hex(0x378ADD), LV_CHART_AXIS_PRIMARY_Y);

    /* Datos iniciales */
    fill_spectrum(&ctx, 0);

    /* Ahora que chart existe, registrar callbacks de checkboxes */
    lv_obj_add_event_cb(cb1, cb_series_cb, LV_EVENT_VALUE_CHANGED, chart);
    lv_obj_add_event_cb(cb2, cb_series_cb, LV_EVENT_VALUE_CHANGED, chart);
    lv_obj_add_event_cb(cb3, cb_series_cb, LV_EVENT_VALUE_CHANGED, chart);

    /* ===== Barra inferior ===== */
    lv_obj_t *status = lv_obj_create(scr);
    lv_obj_set_size(status, 1024, 35);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_radius(status, 0, 0);
    lv_obj_set_style_bg_color(status, lv_color_hex(0x161B22), 0);
    lv_obj_set_style_bg_opa(status, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(status, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(status, lv_color_hex(0x30363D), 0);
    lv_obj_set_style_border_width(status, 1, 0);
    lv_obj_set_style_pad_all(status, 0, 0);
    lv_obj_remove_flag(status, LV_OBJ_FLAG_SCROLLABLE);

    ctx.status_lbl = lv_label_create(status);
    lv_label_set_text(ctx.status_lbl,
        "Device: OES-Sim v2.1   Integration: 100ms   Avg: 16   Range: 380-780 nm");
    lv_obj_center(ctx.status_lbl);
    lv_obj_set_style_text_color(ctx.status_lbl, lv_color_hex(0x8B949E), 0);
    lv_obj_set_style_text_font(ctx.status_lbl, &lv_font_montserrat_14, 0);

    /* ===== Timer de adquisicion (pausado al inicio) ===== */
    ctx.acq_timer = lv_timer_create(acq_timer_cb, 800, &ctx);
    lv_timer_pause(ctx.acq_timer);
}

/* ===== MAIN LVGL ===== */

void lvgl_sw_rotation_main(void)
{
    bsp_display_brightness_init();

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
    };
    i2c_new_master_bus(&i2c_bus_conf, &i2c_handle);

    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");

    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = JD9165_PANEL_BUS_DSI_2CH_CONFIG();

    esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus);

     ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_dbi_io_config_t dbi_config =JD9165_PANEL_IO_DBI_CONFIG();

    esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io);

    esp_lcd_panel_handle_t disp_panel = NULL;
    esp_lcd_dpi_panel_config_t dpi_config = JD9165_1024_600_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);

    dpi_config.num_fbs = LVGL_PORT_LCD_BUFFER_NUMS;

    jd9165_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order = ESP_LCD_COLOR_SPACE_RGB,
        .reset_gpio_num = BSP_LCD_RST,
        .vendor_config = &vendor_config,
    };
    esp_lcd_new_panel_jd9165(io, &lcd_dev_config, &disp_panel);
    esp_lcd_panel_reset(disp_panel);
    esp_lcd_panel_init(disp_panel);

    esp_lcd_dpi_panel_event_callbacks_t cbs = {
#if LVGL_PORT_AVOID_TEAR_MODE
        .on_refresh_done = mipi_dsi_lcd_on_vsync_event,
#else
        .on_color_trans_done = mipi_dsi_lcd_on_vsync_event,
#endif
    };
    esp_lcd_dpi_panel_register_event_callbacks(disp_panel, &cbs, NULL);
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp_handle;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 100000;
    esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle);
     const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LVGL_PORT_H_RES,
        .y_max = LVGL_PORT_V_RES,
        .rst_gpio_num = BSP_LCD_TOUCH_RST, // Shared with LCD reset
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    
    esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp_handle);

    lvgl_port_interface_t interface = (dpi_config.flags.use_dma2d) ? LVGL_PORT_INTERFACE_MIPI_DSI_DMA : LVGL_PORT_INTERFACE_MIPI_DSI_NO_DMA;
    ESP_LOGI(TAG,"interface is %d",interface);
    ESP_ERROR_CHECK(lvgl_port_init(disp_panel, tp_handle, interface));

     bsp_display_brightness_set(100);

    if(lvgl_port_lock(-1))
    {
        // lv_demo_music();
        // lv_demo_benchmark();
        // lv_demo_widgets();
        create_graph_ui();

        lvgl_port_unlock();
    }
}
