/*
 * This file is part of the Bus Pirate project
 * (http://code.google.com/p/the-bus-pirate/).
 *
 * Written and maintained by the Bus Pirate project.
 *
 * To the extent possible under law, the project has waived all copyright and
 * related or neighboring rights to Bus Pirate. This work is published from
 * United States.
 *
 * For details see: http://creativecommons.org/publicdomain/zero/1.0/
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.
 */

#include "ssd1306.h"

#ifdef BP_ENABLE_SSD1306_SUPPORT

#include "bitbang.h"
#include "base.h"
#include "proc_menu.h"

#define SSD1306_DEFAULT_ADDRESS 0x3c
#define OLED_CHARS_PER_LINE     21  // 128 / 6 = 21 full characters safely

#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF

#define SSD1306_SET_CONTRAST            0x81
#define SSD1306_CONTRAST_DEFAULT        0xCF  // Recommended default contrast for most modules

#define SSD1306_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_DISPLAY_ALL_ON          0xA5
#define SSD1306_NORMAL_DISPLAY          0xA6
#define SSD1306_INVERT_DISPLAY          0xA7

#define SSD1306_SET_MULTIPLEX           0xA8
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_DISPLAY_OFFSET          0x00  // No offset

#define SSD1306_SET_START_LINE          0x40  // | line (0-63)
#define SSD1306_START_LINE              0x00  // Start at line 0

#define SSD1306_CHARGE_PUMP             0x8D
#define SSD1306_CHARGE_PUMP_ENABLE      0x14
#define SSD1306_CHARGE_PUMP_DISABLE     0x10

#define SSD1306_MEMORY_ADDR_MODE        0x20
#define SSD1306_HORIZONTAL_MODE         0x00
#define SSD1306_VERTICAL_MODE           0x01
#define SSD1306_PAGE_MODE               0x02

#define SSD1306_SEG_REMAP               0xA0  // normal | 0xA1 remapped
#define SSD1306_SEG_REMAP_INV           0xA1
#define SSD1306_COM_SCAN_NORMAL         0xC0
#define SSD1306_COM_SCAN_REMAP          0xC8

#define SSD1306_SET_COM_PINS            0xDA
#define SSD1306_SET_PRECHARGE           0xD9
#define SSD1306_PRECHARGE_DEFAULT       0xF1  // Recommended default pre-charge period
#define SSD1306_SET_VCOM_DESELECT       0xDB
#define SSD1306_VCOMH_DESELECT_LEVEL    0x40  // ~0.83 × VCC (recommended for good brightness)

#define SSD1306_SET_CLOCK_DIV           0xD5
#define SSD1306_SET_DISPLAY_CLOCKDIV    0x80  // default recommended

#define SSD1306_COLUMN_ADDR             0x21
#define SSD1306_PAGE_ADDR               0x22

#define SSD1306_PAGE_START              0xB0  // | page (0-7)
#define SSD1306_COLUMN_LOW_NIBBLE       0x00  // | low 4 bits
#define SSD1306_COLUMN_HIGH_NIBBLE      0x10  // | high 4 bits

// Control bytes for continuous data/command stream
#define SSD1306_CMD_STREAM              0x00
#define SSD1306_DATA_STREAM             0x40
#define SSD1306_SINGLE_CMD              0x80
#define SSD1306_SINGLE_DATA             0xC0

/*
 * ssd1306xled_font6x8 is by Neven Boyanov
 * ssd1306xled_font8x16 is by Neven Boyanov
 *
 * @created: 2014-08-12
 * @author: Neven Boyanov
 *
 * Copyright (c) 2015 Neven Boyanov, Tinusaur Team. All Rights Reserved.
 * Distributed as open source software under MIT License, see LICENSE.txt file.
 * Please, as a favour, retain the link http://tinusaur.org to The Tinusaur Project.
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */
const unsigned char oled_font6x8[][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // sp
  {0x00, 0x00, 0x00, 0x2f, 0x00, 0x00}, // !
  {0x00, 0x00, 0x07, 0x00, 0x07, 0x00}, // "
  {0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
  {0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
  {0x00, 0x62, 0x64, 0x08, 0x13, 0x23}, // %
  {0x00, 0x36, 0x49, 0x55, 0x22, 0x50}, // &
  {0x00, 0x00, 0x05, 0x03, 0x00, 0x00}, // '
  {0x00, 0x00, 0x1c, 0x22, 0x41, 0x00}, // (
  {0x00, 0x00, 0x41, 0x22, 0x1c, 0x00}, // )
  {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14}, // *
  {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08}, // +
  {0x00, 0x00, 0x00, 0xA0, 0x60, 0x00}, // ,
  {0x00, 0x08, 0x08, 0x08, 0x08, 0x08}, // -
  {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, // .
  {0x00, 0x20, 0x10, 0x08, 0x04, 0x02}, // /
  {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x00, 0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x00, 0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x00, 0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x00, 0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x00, 0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
  {0x00, 0x00, 0x36, 0x36, 0x00, 0x00}, // :
  {0x00, 0x00, 0x56, 0x36, 0x00, 0x00}, // ;
  {0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, // <
  {0x00, 0x14, 0x14, 0x14, 0x14, 0x14}, // =
  {0x00, 0x00, 0x41, 0x22, 0x14, 0x08}, // >
  {0x00, 0x02, 0x01, 0x51, 0x09, 0x06}, // ?
  {0x00, 0x32, 0x49, 0x59, 0x51, 0x3E}, // @
  {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
  {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36}, // B
  {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22}, // C
  {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
  {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41}, // E
  {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01}, // F
  {0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
  {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
  {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00}, // I
  {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01}, // J
  {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41}, // K
  {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40}, // L
  {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
  {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
  {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
  {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06}, // P
  {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
  {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46}, // R
  {0x00, 0x46, 0x49, 0x49, 0x49, 0x31}, // S
  {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01}, // T
  {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
  {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
  {0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
  {0x00, 0x63, 0x14, 0x08, 0x14, 0x63}, // X
  {0x00, 0x07, 0x08, 0x70, 0x08, 0x07}, // Y
  {0x00, 0x61, 0x51, 0x49, 0x45, 0x43}, // Z
  {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, // [
  {0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55}, // 55
  {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
  {0x00, 0x04, 0x02, 0x01, 0x02, 0x04}, // ^
  {0x00, 0x40, 0x40, 0x40, 0x40, 0x40}, // _
  {0x00, 0x00, 0x01, 0x02, 0x04, 0x00}, // '
  {0x00, 0x20, 0x54, 0x54, 0x54, 0x78}, // a
  {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38}, // b
  {0x00, 0x38, 0x44, 0x44, 0x44, 0x20}, // c
  {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F}, // d
  {0x00, 0x38, 0x54, 0x54, 0x54, 0x18}, // e
  {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02}, // f
  {0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // g
  {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78}, // h
  {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00}, // i
  {0x00, 0x40, 0x80, 0x84, 0x7D, 0x00}, // j
  {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00}, // k
  {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00}, // l
  {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78}, // m
  {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78}, // n
  {0x00, 0x38, 0x44, 0x44, 0x44, 0x38}, // o
  {0x00, 0xFC, 0x24, 0x24, 0x24, 0x18}, // p
  {0x00, 0x18, 0x24, 0x24, 0x18, 0xFC}, // q
  {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08}, // r
  {0x00, 0x48, 0x54, 0x54, 0x54, 0x20}, // s
  {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20}, // t
  {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
  {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
  {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
  {0x00, 0x44, 0x28, 0x10, 0x28, 0x44}, // x
  {0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // y
  {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44}, // z
  {0x14, 0x14, 0x14, 0x14, 0x14, 0x14}, // horiz lines
};

extern mode_configuration_t mode_configuration;
extern command_t last_command;

struct _SSD1306_interface {
    unsigned char multiplex_ratio;     // 0x3F for 128x64, 0x1F for 128x32
    unsigned char com_pins_hw_config;  // 0x12 for 128x64, 0x02 for 128x32
    unsigned char pages;               // 8 for 128x64, 4 for 128x32
    unsigned char i2c_address;         // 7-bit address << 1 (write)
    unsigned char cursor_x;            // 0-127 (column)
    unsigned char cursor_y;            // 0 to (pages*8 - 8) — pixel row
} SSD1306;

static void SSD1306_Reset(void);
static void SSD1306_Init(void);
static void SSD1306_Clear(void);
static void SSD1306_Send(unsigned char mode, unsigned char datout);
static void SSD1306_Test(unsigned char first_ch, unsigned char last_ch, int count);


unsigned int OLEDwrite(unsigned int c) {
    if (c >= 32 && c <= 127) {
        unsigned int idx = c - 32;

        // Calculate current page from cursor_y
        unsigned char page = SSD1306.cursor_y / 8;

        // Set position
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_PAGE_START + page);
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_COLUMN_LOW_NIBBLE + (SSD1306.cursor_x & 0x0F));
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_COLUMN_HIGH_NIBBLE + (SSD1306.cursor_x >> 4));

        // Write character
        unsigned char i;
        for (i = 0; i < 6; i++) {
            SSD1306_Send(SSD1306_DATA_STREAM, oled_font6x8[idx][i]);
        }

        // Advance cursor
        SSD1306.cursor_x += 6;
        if (SSD1306.cursor_x >= 128) {
            SSD1306.cursor_x = 0;
            SSD1306.cursor_y += 8;
            if (SSD1306.cursor_y >= (SSD1306.pages * 8)) {
                SSD1306.cursor_y = 0;  // wrap to top
            }
        }
    }
    return 0x100;
}

void OLEDstart(void) {
    //HD44780.RS = HD44780_COMMAND;
    BPMSG1213; // Command mode
}

void OLEDstop(void) {
    //HD44780.RS = HD44780_DATA;
    BPMSG1214; // Data mode
}

void OLEDsetup(void) {
    mode_configuration.high_impedance = ON;
    mode_configuration.command_error = NO;

    SSD1306.i2c_address = SSD1306_DEFAULT_ADDRESS << 1;

    BPMSG1215; // "I2C address (default 0x27):"
    SSD1306.i2c_address = getnumber(SSD1306_DEFAULT_ADDRESS, 0, 255, 0) << 1;

    bitbang_setup(2, BITBANG_SPEED_100KHZ); //2wire mode, 100kHz (PCF8574 max)
    
    BPMSG1216; // Adapter ready message
}

void OLEDsetup_exc(void) {
    
}

void OLEDmacro(unsigned int c) {
    int input;

    consumewhitechars();
    if (cmdbuf[cmdstart] == ')') {
        cmdstart = (cmdstart + 1) & CMDLENMSK;
    }

    consumewhitechars();
    input = getint();

    mode_configuration.command_error = NO;
    
    switch (c) {
        case 0:
            BPMSG1219; // macro menu
            break;
        case 1: // reset only
            BPMSG1093;
            SSD1306_Reset();
            break;
        case 2:
            BPMSG1093;
            SSD1306_Reset();

            input = getnumber(1, 1, 2, 0);

            if (input == 1) {
                // 128x64
                SSD1306.multiplex_ratio = 0x3F;
                SSD1306.com_pins_hw_config = 0x12;
                SSD1306.pages = 8;
            } else {
                // 128x32
                SSD1306.multiplex_ratio = 0x1F;
                SSD1306.com_pins_hw_config = 0x02;
                SSD1306.pages = 4;
            }
            SSD1306_Init();
            SSD1306_Clear();
            BPMSG1221;
            break;
        case 3:
            SSD1306_Clear();
            BPMSG1222;
            break;
        case 4:
            input = getint();
            if (input < 0) input = 0;

            unsigned int total_chars = OLED_CHARS_PER_LINE * SSD1306.pages;

            if ((unsigned int)input >= total_chars) { input = total_chars - 1; }
            
            SSD1306.cursor_x = (input % OLED_CHARS_PER_LINE) * 6;
            SSD1306.cursor_y = (input / OLED_CHARS_PER_LINE) * 8;
    
            BPMSG1223;
            break;
        case 8: // numbers test
            SSD1306_Test(0x30, 0x39, input); //0 to 9
            break;
        case 9: // characters test
            SSD1306_Test(0x21, 0x7E, input); //! to ~
            break;
        default:
            MSG_UNKNOWN_MACRO_ERROR;
    }
}

void OLEDpins(void) {
    MSG_I2C_PINS_STATE;
}

void SSD1306_Init(void) {
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_DISPLAY_OFF);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_CLOCK_DIV);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_DISPLAY_CLOCKDIV);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_MULTIPLEX);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306.multiplex_ratio);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_DISPLAY_OFFSET);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_DISPLAY_OFFSET);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_START_LINE | SSD1306_START_LINE);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_CHARGE_PUMP);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_CHARGE_PUMP_ENABLE);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_MEMORY_ADDR_MODE);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_PAGE_MODE);  // Using page mode for simplicity

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SEG_REMAP_INV);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_COM_SCAN_REMAP);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_COM_PINS);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306.com_pins_hw_config);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_CONTRAST);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_CONTRAST_DEFAULT);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_PRECHARGE);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_PRECHARGE_DEFAULT);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_SET_VCOM_DESELECT);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_VCOMH_DESELECT_LEVEL);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_DISPLAY_ALL_ON_RESUME);
    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_NORMAL_DISPLAY);

    SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_DISPLAY_ON);

    bp_delay_ms(100);
}

static void SSD1306_Clear(void) {
    unsigned char page, col;

    for (page = 0; page < SSD1306.pages; page++) {
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_PAGE_START + page);
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_COLUMN_LOW_NIBBLE);
        SSD1306_Send(SSD1306_CMD_STREAM, SSD1306_COLUMN_HIGH_NIBBLE);

        for (col = 0; col < 128; col++) {
            SSD1306_Send(SSD1306_DATA_STREAM, 0x00);
        }
    }

    SSD1306.cursor_x = 0;
    SSD1306.cursor_y = 0;
}

void SSD1306_Reset(void) {

}

static void SSD1306_Send(unsigned char mode, unsigned char datout) {
    bitbang_i2c_start(BITBANG_I2C_START_ONE_SHOT);
    bitbang_write_value(SSD1306.i2c_address);
    if (bitbang_read_bit() == 1) { MSG_NACK; return; }
    bitbang_write_value(mode);
    if (bitbang_read_bit() == 1) { MSG_NACK; return; }
    bitbang_write_value(datout);
    if (bitbang_read_bit() == 1) { MSG_NACK; return; }
    bitbang_i2c_stop();
}

static void SSD1306_Test(unsigned char first_ch, unsigned char last_ch, int count) {
    unsigned char ch;
    int i;

    //clear display
    bp_delay_ms(15);

    if (count == 0) count = 80;  // Default

    ch = first_ch;
    for (i = 0; i < count; i++) {
        if (ch > last_ch) ch = first_ch;
        OLEDwrite(ch);
        user_serial_transmit_character(ch);
        ch++;
    }
}

#endif /* BP_ENABLE_SSD1306_SUPPORT */