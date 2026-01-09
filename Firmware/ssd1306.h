/*
 * This file is part of the Bus Pirate project (http://code.google.com/p/the-bus-pirate/).
 *
 * Written and maintained by the Bus Pirate project.
 *
 * To the extent possible under law, the project has
 * waived all copyright and related or neighboring rights to Bus Pirate. This
 * work is published from United States.
 *
 * For details see: http://creativecommons.org/publicdomain/zero/1.0/.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef BP_SSD1306_H
#define BP_SSD1306_H

#include "configuration.h"

#ifdef BP_ENABLE_SSD1306_SUPPORT

unsigned int OLEDread(void);
unsigned int OLEDwrite(unsigned int c);
void OLEDstart(void);
void OLEDstop(void);
void OLEDsetup(void);
void OLEDsetup_exc(void);
void OLEDmacro(unsigned int c);
void OLEDpins(void);

#endif /* BP_ENABLE_SSD1306_SUPPORT */
#endif /* !BP_SSD1306_H */
