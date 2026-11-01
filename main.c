/*
 * Pintor de pixels - demo de depuração via SWD na BitDogLab (RP2040)
 *
 *  Joystick (eixos, ADC) : move o cursor na matriz 5x5
 *  Botão A               : pinta o pixel sob o cursor com a cor atual
 *  Botão B               : apaga o pixel sob o cursor
 *  Clique do joystick    : troca a cor atual (vermelho -> verde -> azul)
 *
 *  BUG=1 : versão com o bug. Cada *_init() registra o seu próprio callback de
 *          GPIO. O RP2040 tem um único callback de GPIO por núcleo, então o
 *          registro do joystick substitui o dos botões: A e B passam a cair no
 *          handler do clique do joystick.
 *  BUG=0 : versão corrigida (um único dispatcher que confere o GPIO).
 *
 * Confira os pinos no esquemático da sua revisão da BitDogLab.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "hardware/sync.h"

#ifndef BUG
#define BUG 1
#endif

/* ---------- Pinos ---------- */
#define LED_MATRIX_PIN 7
#define BTN_A          5
#define BTN_B          6
#define JOY_SW         22
#define JOY_X_PIN      27   /* ADC1 */
#define JOY_Y_PIN      26   /* ADC0 */

/* Se o cursor andar ao contrário, troque para 1 */
#define INVERT_X 0
#define INVERT_Y 0

/* ---------- Parâmetros ---------- */
#define GRID           5
#define ADC_CENTER     2048
#define ADC_DEADZONE   900
#define MOVE_REPEAT_MS 200
#define DEBOUNCE_US    40000u
#define BLINK_MS       250

/* Índice físico do LED para cada (linha, coluna), linha 0 = topo.
 * Se a imagem sair espelhada ou invertida, ajuste esta tabela. */
static const uint8_t LED_MAP[GRID][GRID] = {
    {24, 23, 22, 21, 20},
    {15, 16, 17, 18, 19},
    {14, 13, 12, 11, 10},
    { 5,  6,  7,  8,  9},
    { 4,  3,  2,  1,  0},
};

/* Cores (R,G,B). Índice 0 = vazio. Baixo brilho de propósito. */
static const uint8_t DIM[4][3]    = {{0, 0, 0}, {12, 0, 0}, {0, 12, 0}, {0, 0, 12}};
static const uint8_t BRIGHT[4][3] = {{0, 0, 0}, {60, 0, 0}, {0, 60, 0}, {0, 0, 60}};

/* ---------- Estado ---------- */
static uint8_t canvas[GRID][GRID];
static int cx = GRID / 2, cy = GRID / 2;
static uint8_t cur_color = 1;

/* Eventos sinalizados pelas ISRs e consumidos no main */
static volatile bool evt_paint;
static volatile bool evt_erase;
static volatile bool evt_color;

static PIO  led_pio;
static uint led_sm;

/* ---------- Callbacks de GPIO ---------- */
static inline bool debounced(uint32_t *last_us) {
    uint32_t now = time_us_32();
    if ((uint32_t)(now - *last_us) < DEBOUNCE_US) return false;
    *last_us = now;
    return true;
}

__attribute__((noinline))
static void on_button(uint gpio, uint32_t events) {
    (void)events;
    static uint32_t last_a, last_b;
    if (gpio == BTN_A) {
        if (debounced(&last_a)) evt_paint = true;
    } else if (gpio == BTN_B) {
        if (debounced(&last_b)) evt_erase = true;
    }
}

/* Assume que só o clique do joystick chega aqui e por isso não confere o gpio. */
__attribute__((noinline))
static void on_joy_click(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    static uint32_t last;
    if (debounced(&last)) evt_color = true;
}

#if BUG
    #define BUTTONS_CALLBACK on_button
    #define JOY_CALLBACK     on_joy_click
#else
/* Correção: um único callback que despacha pelo número do GPIO */
__attribute__((noinline))
static void on_gpio_irq(uint gpio, uint32_t events) {
    switch (gpio) {
    case BTN_A:
    case BTN_B:
        on_button(gpio, events);
        break;
    case JOY_SW:
        on_joy_click(gpio, events);
        break;
    default:
        break;
    }
}
    #define BUTTONS_CALLBACK on_gpio_irq
    #define JOY_CALLBACK     on_gpio_irq
#endif

/* ---------- Inicialização ---------- */
static void buttons_init(void) {
    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);

    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &BUTTONS_CALLBACK);
    gpio_set_irq_enabled(BTN_B, GPIO_IRQ_EDGE_FALL, true);
}

static void joystick_init(void) {
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);

    gpio_init(JOY_SW);
    gpio_set_dir(JOY_SW, GPIO_IN);
    gpio_pull_up(JOY_SW);

    gpio_set_irq_enabled_with_callback(JOY_SW, GPIO_IRQ_EDGE_FALL, true, &JOY_CALLBACK);
}

static void matrix_init(void) {
    led_pio = pio0;
    led_sm = pio_claim_unused_sm(led_pio, true);
    uint offset = pio_add_program(led_pio, &ws2812_program);
    ws2812_program_init(led_pio, led_sm, offset, LED_MATRIX_PIN, 800000, false);
}

/* ---------- Joystick (polling do ADC) ---------- */
static void joy_read(int *dx, int *dy) {
    adc_select_input(JOY_X_PIN - 26);
    int x = adc_read();
    adc_select_input(JOY_Y_PIN - 26);
    int y = adc_read();

    int mx = 0, my = 0;
    if (x > ADC_CENTER + ADC_DEADZONE) mx = +1;
    if (x < ADC_CENTER - ADC_DEADZONE) mx = -1;
    if (y > ADC_CENTER + ADC_DEADZONE) my = -1;   /* valor alto = para cima */
    if (y < ADC_CENTER - ADC_DEADZONE) my = +1;

    *dx = INVERT_X ? -mx : mx;
    *dy = INVERT_Y ? -my : my;
}

/* ---------- Matriz ---------- */
static inline uint32_t pack_grb(const uint8_t rgb[3]) {
    return ((uint32_t)rgb[1] << 16) | ((uint32_t)rgb[0] << 8) | (uint32_t)rgb[2];
}

static void render(bool cursor_on) {
    uint32_t phys[GRID * GRID];

    for (int r = 0; r < GRID; r++) {
        for (int c = 0; c < GRID; c++) {
            const uint8_t *col = DIM[canvas[r][c]];
            if (cursor_on && r == cy && c == cx) col = BRIGHT[cur_color];
            phys[LED_MAP[r][c]] = pack_grb(col);
        }
    }
    for (int i = 0; i < GRID * GRID; i++) {
        pio_sm_put_blocking(led_pio, led_sm, phys[i] << 8);
    }
}

/* ---------- Utilitários ---------- */
static bool take(volatile bool *flag) {
    uint32_t s = save_and_disable_interrupts();
    bool v = *flag;
    *flag = false;
    restore_interrupts(s);
    return v;
}

static int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---------- main ---------- */
int main(void) {
    stdio_init_all();

    matrix_init();
    buttons_init();
    joystick_init();

    printf("Pintor de pixels - BUG=%d\n", BUG);

    bool blink = true;
    bool dirty = true;
    absolute_time_t next_blink = make_timeout_time_ms(BLINK_MS);
    uint32_t last_move_ms = 0;

    while (true) {
        int dx, dy;
        joy_read(&dx, &dy);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((dx || dy) && (now_ms - last_move_ms) >= MOVE_REPEAT_MS) {
            cx = clampi(cx + dx, 0, GRID - 1);
            cy = clampi(cy + dy, 0, GRID - 1);
            last_move_ms = now_ms;
            dirty = true;
        }

        if (take(&evt_paint)) {
            canvas[cy][cx] = cur_color;
            dirty = true;
        }
        if (take(&evt_erase)) {
            canvas[cy][cx] = 0;
            dirty = true;
        }
        if (take(&evt_color)) {
            cur_color = (uint8_t)(cur_color % 3 + 1);
            dirty = true;
        }

        if (time_reached(next_blink)) {
            blink = !blink;
            next_blink = delayed_by_ms(next_blink, BLINK_MS);
            dirty = true;
        }

        if (dirty) {
            render(blink);
            dirty = false;
        }

        sleep_ms(10);
    }
}
