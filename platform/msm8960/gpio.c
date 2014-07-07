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

#include <debug.h>
#include <reg.h>
#include <platform/iomap.h>
#include <platform/gpio.h>
#include <gsbi.h>
#include <dev/pm8921.h>
#include <sys/types.h>
#include <smem.h>
#include <board.h>

typedef struct gpioregs gpioregs;

struct gpioregs
{
	unsigned out;
	unsigned in;
	unsigned oe;
};

static gpioregs GPIO_REGS[] = {
	{
		.out =  GPIO_OUT_0,
		.in =   GPIO_IN_0,
		.oe =   GPIO_OE_0,
	},
	{
		.out =  GPIO_OUT_1,
		.in =   GPIO_IN_1,
		.oe =   GPIO_OE_1,
	},
	{
		.out =  GPIO_OUT_2,
		.in =   GPIO_IN_2,
		.oe =   GPIO_OE_2,
	},
};

static gpioregs *find_gpio(unsigned n, unsigned *bit)
{
	if (n > 89) {
		dprintf(CRITICAL, "Wrong GPIO %d\n", n);
		return 0;
	}
	if (n > 63) {
		*bit = 1 << (n - 64);
		return GPIO_REGS + 2;
	}
	if (n > 31) {
		*bit = 1 << (n - 32);
		return GPIO_REGS + 1;
	}
	*bit = 1 << n;
	return GPIO_REGS + 0;
}

int gpio_direction(unsigned n, unsigned flags)
{
        gpioregs *r;
        unsigned b;
        unsigned v;

        if ((r = find_gpio(n, &b)) == 0)
                return -1;

        v = readl(r->oe);
        if (flags & GPIO_OUTPUT)
                writel(v | b, r->oe);
        else
                writel(v & (~b), r->oe);
        return 0;
}

void gpio_output_value(unsigned n, unsigned on)
{
        gpioregs *r;
        unsigned b;
        unsigned v;

        if ((r = find_gpio(n, &b)) == 0)
		return;

        v = readl(r->out);
        if (on)
                writel(v | b, r->out);
        else
                writel(v & (~b), r->out);
}

int gpio_input_value(unsigned n)
{
        gpioregs *r;
        unsigned b;

        if ((r = find_gpio(n, &b)) == 0)
		return 0;

        return (readl(r->in) & b) ? 1 : 0;
}

void gpio_tlmm_config(uint32_t gpio, uint8_t func,
		      uint8_t dir, uint8_t pull,
		      uint8_t drvstr, uint32_t enable)
{
	unsigned int val = 0;
	val |= pull;
	val |= func << 2;
	val |= drvstr << 6;
	val |= enable << 9;
	unsigned int *addr = (unsigned int *)GPIO_CONFIG_ADDR(gpio);
	writel(val, addr);
	return;
}

void gpio_set(uint32_t gpio, uint32_t dir)
{
	unsigned int *addr = (unsigned int *)GPIO_IN_OUT_ADDR(gpio);
	writel(dir, addr);
	return;
}

uint32_t gpio_get(uint32_t gpio)
{
	unsigned int *addr = (unsigned int *)GPIO_IN_OUT_ADDR(gpio);
	return readl(addr) ? 1 : 0;
}

/* TODO: this and other code below in this file should ideally by in target dir.
 * keeping it here for this brigup.
 */

/* Configure gpio for uart - based on gsbi id */
#define MSM_BOOT_UART_SWITCH_GPIO_MITWOA	33
#define MSM_BOOT_UART_SWITCH_GPIO_MITWO		62
void gpio_config_uart_dm(uint8_t id)
{

	if(board_target_id() == LINUX_MACHTYPE_8960_CDP)
	{
		gpio_tlmm_config(MSM_BOOT_UART_SWITCH_GPIO_MITWOA, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_16MA, GPIO_ENABLE);
		gpio_direction(MSM_BOOT_UART_SWITCH_GPIO_MITWOA, GPIO_OUTPUT);
		gpio_set(MSM_BOOT_UART_SWITCH_GPIO_MITWOA, GPIO_IN_OUT_HIGH);

	} else if (board_target_id() == LINUX_MACHTYPE_8064_MTP
			|| board_target_id() == LINUX_MACHTYPE_8064_MITWO) {
		gpio_tlmm_config(MSM_BOOT_UART_SWITCH_GPIO_MITWO, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_16MA, GPIO_ENABLE);
		gpio_direction(MSM_BOOT_UART_SWITCH_GPIO_MITWO, GPIO_OUTPUT);
		gpio_set(MSM_BOOT_UART_SWITCH_GPIO_MITWO, GPIO_IN_OUT_HIGH);
	}

	if (board_platform_id() == MPQ8064 || board_platform_id() == MSM8260AB) {
		switch (id) {

		case GSBI_ID_5:
			/* configure rx gpio */
			gpio_tlmm_config(23, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(22, 1, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;

		default:
			ASSERT(0);
		}
	}
	else if((board_platform_id() == APQ8064) ||
			(board_platform_id() == APQ8064AB))
	{
		switch (id) {

		case GSBI_ID_1:
			/* configure rx gpio */
			gpio_tlmm_config(19, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(18, 1, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;


		case GSBI_ID_7:
			/* configure rx gpio */
			gpio_tlmm_config(83, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(82, 2, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;

		default:
			ASSERT(0);
		}
	}
	else
	{
		switch (id) {

		case GSBI_ID_3:
			/* configure rx gpio */
			gpio_tlmm_config(15, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(14, 1, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;

		case GSBI_ID_5:
			/* configure rx gpio */
			gpio_tlmm_config(23, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(22, 1, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;

		case GSBI_ID_8:
			/* configure rx gpio */
			gpio_tlmm_config(35, 1, GPIO_INPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			/* configure tx gpio */
			gpio_tlmm_config(34, 1, GPIO_OUTPUT, GPIO_NO_PULL,
							 GPIO_8MA, GPIO_DISABLE);
			break;

		default:
			ASSERT(0);
		}
	}
}

/* Configure gpio for i2c - based on gsbi id */
void gpio_config_i2c(uint8_t id)
{
        switch (id) {
        case GSBI_ID_10:
                gpio_tlmm_config(73, 2, GPIO_OUTPUT, GPIO_NO_PULL,
                                 GPIO_2MA, GPIO_DISABLE);
                gpio_tlmm_config(74, 2, GPIO_OUTPUT, GPIO_NO_PULL,
                                 GPIO_2MA, GPIO_DISABLE);
        case GSBI_ID_1:
                gpio_tlmm_config(20, 1, GPIO_OUTPUT, GPIO_NO_PULL,
                                 GPIO_2MA, GPIO_DISABLE);
                gpio_tlmm_config(21, 1, GPIO_OUTPUT, GPIO_NO_PULL,
                                 GPIO_2MA, GPIO_DISABLE);
                break;
        default:
                ASSERT(0);
        }
}

struct pm8xxx_gpio_init {
	uint32_t gpio;
	struct pm8921_gpio config;
};

#define PM8XXX_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= _gpio, \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8XXX_GPIO_OUTPUT(_gpio, _val) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, 0, _val, \
			PM_GPIO_PULL_NO, 2, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 1, 0)


#define PM8XXX_GPIO_INPUT(_gpio, _pull) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, \
			_pull, 2, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 1, 0)

#define PM8XXX_GPIO_INPUT_POL(_gpio, _pull) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, \
			_pull, 2, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

/* Initial pm8038 GPIO configurations */
static struct pm8xxx_gpio_init pm8038_keypad_gpios[] = {
	/* keys GPIOs */
	PM8XXX_GPIO_INPUT(PM_GPIO(3), PM_GPIO_PULL_UP_30),
	PM8XXX_GPIO_INPUT(PM_GPIO(8), PM_GPIO_PULL_UP_30),
	PM8XXX_GPIO_INPUT(PM_GPIO(10), PM_GPIO_PULL_UP_30),
	PM8XXX_GPIO_INPUT(PM_GPIO(11), PM_GPIO_PULL_UP_30),
};

static struct pm8xxx_gpio_init pm8921_keypad_gpios[] = {
	/* keys GPIOs */
	PM8XXX_GPIO_INPUT(PM_GPIO(1), PM_GPIO_PULL_UP_31_5),
	PM8XXX_GPIO_INPUT(PM_GPIO(2), PM_GPIO_PULL_UP_31_5),
	PM8XXX_GPIO_OUTPUT(PM_GPIO(9), 0),
	PM8XXX_GPIO_OUTPUT(PM_GPIO(10), 0),
};

/* pm8921 GPIO configuration for APQ8064 keypad */
static struct pm8xxx_gpio_init pm8921_keypad_gpios_apq[] = {
	/* keys GPIOs */
	PM8XXX_GPIO_INPUT(PM_GPIO(1), PM_GPIO_PULL_UP_31_5),
	PM8XXX_GPIO_INPUT(PM_GPIO(2), PM_GPIO_PULL_UP_31_5),
	PM8XXX_GPIO_OUTPUT(PM_GPIO(9), 0),
	PM8XXX_GPIO_OUTPUT(PM_GPIO(10), 0),
};

void msm8960_keypad_gpio_init()
{
		int i = 0;
		int num = 0;

		num = ARRAY_SIZE(pm8921_keypad_gpios);

		for(i=0; i < num; i++)
		{
			pm8921_gpio_config(pm8921_keypad_gpios[i].gpio,
								&(pm8921_keypad_gpios[i].config));
		}
}

void msm8930_keypad_gpio_init()
{
		int i = 0;
		int num = 0;

		num = ARRAY_SIZE(pm8038_keypad_gpios);

		for(i=0; i < num; i++)
		{
			pm8921_gpio_config(pm8038_keypad_gpios[i].gpio,
								&(pm8038_keypad_gpios[i].config));
		}
}

void apq8064_keypad_gpio_init()
{
		int i = 0;
		int num = 0;

		num = ARRAY_SIZE(pm8921_keypad_gpios_apq);

		for(i=0; i < num; i++)
		{
			pm8921_gpio_config(pm8921_keypad_gpios_apq[i].gpio,
								&(pm8921_keypad_gpios_apq[i].config));
		}
}

void pmic8921_gpio_set(uint32_t gpio, uint32_t level)
{
	struct pm8xxx_gpio_init pm8921_gpio[] = {
		PM8XXX_GPIO_OUTPUT(PM_GPIO(gpio), level),
	};

	pm8921_gpio_config(pm8921_gpio[0].gpio,
		&(pm8921_gpio[0].config));

	return;
}

uint32_t pmic8921_gpio_get(uint32_t gpio)
{
	int ret = 0;
	uint8_t status = 0;

	ret = pm8921_gpio_get(PM_GPIO(gpio), &status);
	if (ret) {
		status = 0xFFFFFFFF;
	}
	dprintf(CRITICAL, "pmic8921_gpio_get ret %d status %d\n", ret, status);

	return status;
}

#define PM8921_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, 0, _val, \
			PM_GPIO_PULL_NO, 2, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)


#define PM8921_GPIO_OUTPUT_BUFCONF(_gpio, _val, _strength, _bufconf) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT,\
			PM_GPIO_OUT_BUF_##_bufconf, _val, \
			PM_GPIO_PULL_NO, 2, \
			PM_GPIO_STRENGTH_##_strength, \
			PM_GPIO_FUNC_NORMAL, 0, 0)


static struct pm8xxx_gpio_init pm8921_display_gpios_apq[] = {
	/* Display GPIOs */
	/* Bl: ON, PWM mode */
	PM8921_GPIO_OUTPUT_FUNC(PM_GPIO(26), 1, PM_GPIO_FUNC_2),
	/* LCD1_PWR_EN_N */
	PM8921_GPIO_OUTPUT_BUFCONF(PM_GPIO(36), 0, LOW, OPEN_DRAIN),
	/* DISP_RESET_N */
	PM8921_GPIO_OUTPUT_BUFCONF(PM_GPIO(25), 1, LOW, CMOS),
};

void apq8064_display_gpio_init()
{
		int i = 0;
		int num = 0;

		num = ARRAY_SIZE(pm8921_display_gpios_apq);

		for (i = 0; i < num; i++) {
			pm8921_gpio_config(pm8921_display_gpios_apq[i].gpio,
				&(pm8921_display_gpios_apq[i].config));
		}
}

static struct pm8xxx_gpio_init mi_display_gpios_apq[] = {
	PM8XXX_GPIO_OUTPUT(PM_GPIO(11), 0),
	PM8XXX_GPIO_OUTPUT(PM_GPIO(25), 0),
	PM8XXX_GPIO_INPUT_POL(PM_GPIO(12), PM_GPIO_PULL_NO),
};

void mi_display_gpio_init()
{
	int i = 0;
	int num = 0;

	num = ARRAY_SIZE(mi_display_gpios_apq);

	for (i = 0; i < num; i++) {
		pm8921_gpio_config(mi_display_gpios_apq[i].gpio,
			&(mi_display_gpios_apq[i].config));
	}
}
