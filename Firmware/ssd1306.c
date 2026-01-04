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

#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF

#define SSD1306_SET_CONTRAST            0x81

#define SSD1306_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_DISPLAY_ALL_ON          0xA5
#define SSD1306_NORMAL_DISPLAY          0xA6
#define SSD1306_INVERT_DISPLAY          0xA7

#define SSD1306_SET_MULTIPLEX           0xA8
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_SET_START_LINE          0x40  // | line (0-63)

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
#define SSD1306_SET_VCOM_DESELECT       0xDB

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

#endif /* BP_ENABLE_SSD1306_SUPPORT */