/* Copyright 2021 Jay Greco
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#include "raw_hid.h"
#include "oled_logo.h"

enum layer_names {
  _MA,
  _FN
};

enum custom_keycodes {
    KC_CUST = SAFE_RANGE,
    KC_OLE0,
    KC_OLE1,
    KC_OLE2
};

enum oled_mode {
    OLED_MODE_LOGO,
    OLED_MODE_INTERNAL,
    OLED_MODE_RAWHID
};

typedef enum {
    HID_OP_SETBUF
} hid_op_t;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_MA] = LAYOUT_ansi(
                 KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_HOME,
        XXXXXXX, KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS, KC_DEL,
        KC_GRV,  KC_LCTL, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,  KC_PGUP,
        XXXXXXX, KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,          KC_UP,   KC_PGDN,
        XXXXXXX, KC_CAPS, KC_LALT, KC_LGUI,                   KC_SPC,                    KC_RALT, MO(_FN), KC_RCTL, KC_LEFT,          KC_DOWN, KC_RGHT
    ),
    [_FN] = LAYOUT_ansi(
                 KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______,  KC_END,
        RGB_TOG, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
        KC_OLE0, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______,
        KC_OLE1, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______, _______,
        KC_OLE2, _______, _______, _______,                   _______,                   _______, _______, _______, KC_MPRV,          KC_MPLY, KC_MNXT
    ),
};

#define OLED_BUFSIZ 512

static char oled_buf[OLED_BUFSIZ] = { 0 };
static enum oled_mode g_oled_mode = OLED_MODE_INTERNAL;
static int oled_flush = 1;

static uint8_t hue_raw = 0;
static uint16_t hue_360 = 0;
static char hue_str[4] = { 0 };

void digits(uint16_t value, char str[4]) {
    str[3] = 0;
    str[2] = '0' + value % 10;
    str[1] = '0' + (value /= 10) % 10;
    str[0] = '0' + (value /= 10) % 10;
}

void update_rgblight_state(void) {
    hue_raw = rgblight_get_hue();
    hue_360 = (uint16_t)(((uint32_t)hue_raw * 360) / 256);
    digits(hue_360, hue_str);
}

void keyboard_post_init_user(void) {
    update_rgblight_state();
}

#ifdef OLED_ENABLE
oled_rotation_t oled_init_user(oled_rotation_t rotation) { return OLED_ROTATION_180; }

void oled_render_logo(void) {
    oled_write_raw_P(oled_logo, sizeof(oled_logo));

    oled_flush = 0;
}

void oled_render_internal(void) {
    oled_write_P(PSTR("Layer: "), false);

    switch (get_highest_layer(layer_state)) {
        case _MA:
            oled_write_P(PSTR("Default\n"), false);
            break;
        case _FN:
            oled_write_P(PSTR("FN\n"), true);
            break;
        default:
            oled_write_P(PSTR("Undefined\n"), false);
            break;
    }

    char wpm_str[4] = { 0 };
    digits(get_current_wpm(), wpm_str);
    oled_write_P(PSTR("Current WPM: "), false);
    oled_write_ln(wpm_str, false);

    oled_write_P(PSTR("RGB Light Hue: "), false);
    oled_write_ln(hue_str, false);
}

void oled_render_rawhid(void) {
    oled_write_raw(oled_buf, OLED_BUFSIZ);

    oled_flush = 0;
}

bool oled_task_user(void) {
    if (!oled_flush) return false;

    switch (g_oled_mode) {
        case OLED_MODE_LOGO:
            oled_render_logo();
            break;
        case OLED_MODE_INTERNAL:
            oled_render_internal();
            break;
        case OLED_MODE_RAWHID:
            oled_render_rawhid();
            break;
    }

    return false;
}
#endif

void raw_hid_receive(uint8_t *data, uint8_t length) {
    uint8_t inst = data[0];
    uint8_t flag_flush = inst & 0x80;
    uint8_t opcode = inst & 0x0F;

    switch (opcode) {
        case HID_OP_SETBUF:
            memcpy(oled_buf + (int)data[1] * 16, &data[2], 16);
            break;
        default:
            break;
    }

    if (flag_flush) {
        oled_flush = 1;
    }

    raw_hid_send(data, length);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  // Send keystrokes to host keyboard, if connected (see readme)
    process_record_remote_kb(keycode, record);
    switch(keycode) {
        case KC_CUST: //custom macro
            if (record->event.pressed) {
            }
            break;

        case KC_OLE0:
            if (record->event.pressed) {
                g_oled_mode = OLED_MODE_LOGO;
                oled_flush = 1;
            }
            break;

        case KC_OLE1:
            if (record->event.pressed) {
                oled_clear();
                g_oled_mode = OLED_MODE_INTERNAL;
                oled_flush = 1;
            }
            break;

        case KC_OLE2:
            if (record->event.pressed) {
                g_oled_mode = OLED_MODE_RAWHID;
                oled_flush = 1;
            }
            break;

        case RM_1: //remote macro 1
            if (record->event.pressed) {
            }
            break;

        case RM_2: //remote macro 2
            if (record->event.pressed) {
            }
            break;

        case RM_3: //remote macro 3
            if (record->event.pressed) {
            }
            break;

        case RM_4: //remote macro 4
            if (record->event.pressed) {
            }
            break;

    }
    return true;
}


bool encoder_update_user(uint8_t index, bool clockwise) {
    if (IS_LAYER_ON(_FN)) {
        if (clockwise) {
            rgblight_increase_hue();
        } else {
            rgblight_decrease_hue();
        }
        update_rgblight_state();
    } else {
        if (clockwise) {
            tap_code(KC_VOLU);
        } else {
            tap_code(KC_VOLD);
        }
    }
    return false;
}

void matrix_init_user(void) {
    // Initialize remote keyboard, if connected (see readme)
    matrix_init_remote_kb();
}

void matrix_scan_user(void) {
    // Scan and parse keystrokes from remote keyboard, if connected (see readme)
    matrix_scan_remote_kb();
}
