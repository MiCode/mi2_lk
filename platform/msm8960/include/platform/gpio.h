/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2011-2014, Xiaomi Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLATFORM_MSM8960_GPIO_H
#define __PLATFORM_MSM8960_GPIO_H

/* GPIO TLMM: Direction */
#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

/* GPIO TLMM: Pullup/Pulldown */
#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_KEEPER     2
#define GPIO_PULL_UP    3

/* GPIO TLMM: Drive Strength */
#define GPIO_2MA        0
#define GPIO_4MA        1
#define GPIO_6MA        2
#define GPIO_8MA        3
#define GPIO_10MA       4
#define GPIO_12MA       5
#define GPIO_14MA       6
#define GPIO_16MA       7

/* GPIO TLMM: Status */
#define GPIO_ENABLE     1
#define GPIO_DISABLE    0

#define GPIO_BASE_REG(x) (TLMM_BASE_ADDR + 0x3000 + (x))

/* output value */
#define GPIO_OUT_0      GPIO_BASE_REG(0x00) /* gpio  31-0   */
#define GPIO_OUT_1      GPIO_BASE_REG(0x04) /* gpio  63-31  */
#define GPIO_OUT_2      GPIO_BASE_REG(0x08) /* gpio  89-64  */

/* same pin map as above, output enable */
#define GPIO_OE_0       GPIO_BASE_REG(0x80)
#define GPIO_OE_1       GPIO_BASE_REG(0x84)
#define GPIO_OE_2       GPIO_BASE_REG(0x88)

/* same pin map as above, input read */
#define GPIO_IN_0       GPIO_BASE_REG(0x60)
#define GPIO_IN_1       GPIO_BASE_REG(0x64)
#define GPIO_IN_2       GPIO_BASE_REG(0x68)

/* GPIO_IN_OUT: OUTPUT LEVEL */
#define GPIO_IN_OUT_HIGH	0x2
#define GPIO_IN_OUT_LOW		0x00

void gpio_config_i2c(uint8_t gsbi_id);
void gpio_config_uart_dm(uint8_t id);
void msm8960_keypad_gpio_init();
void msm8930_keypad_gpio_init();
void pmic8921_gpio_set(uint32_t gpio, uint32_t level);
uint32_t pmic8921_gpio_get(uint32_t gpio);
#endif
