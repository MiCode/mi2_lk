/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2011-2014, Xiaomi Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 *
 */

#include <debug.h>
#include <board.h>
#include <smem.h>
#include <baseband.h>

static struct board_data board = {UNKNOWN,
	HW_PLATFORM_UNKNOWN,
	HW_PLATFORM_SUBTYPE_UNKNOWN,
	LINUX_MACHTYPE_UNKNOWN,
	BASEBAND_MSM,
	HW_PLATFORM_P0,
	};

static void platform_detect()
{
	struct smem_board_info_v6 board_info_v6;
	struct smem_board_info_v7 board_info_v7;
	unsigned int board_info_len = 0;
	unsigned ret = 0;
	unsigned format = 0;

	ret = smem_read_alloc_entry_offset(SMEM_BOARD_INFO_LOCATION,
						   &format, sizeof(format), 0);
	if (ret)
		return;

	if (format == 6)
	{
			board_info_len = sizeof(board_info_v6);

		ret = smem_read_alloc_entry(SMEM_BOARD_INFO_LOCATION,
				&board_info_v6,
				board_info_len);
		if (ret)
			return;

		board.platform = board_info_v6.board_info_v3.msm_id;
		board.soc_id = board_info_v6.board_info_v3.msm_id;
		board.soc_version = board_info_v6.board_info_v3.msm_version;
		board.platform_hw = board_info_v6.board_info_v3.hw_platform;
		board.platform_subtype = board_info_v6.platform_subtype;
		board.platform_version = board_info_v6.platform_version;
	}
	else if (format == 7)
	{
		board_info_len = sizeof(board_info_v7);

		ret = smem_read_alloc_entry(SMEM_BOARD_INFO_LOCATION,
				&board_info_v7,
				board_info_len);
		if (ret)
			return;

		board.platform = board_info_v7.board_info_v3.msm_id;
		board.soc_id = board_info_v7.board_info_v3.msm_id;
		board.soc_version = board_info_v7.board_info_v3.msm_version;
		board.platform_hw = board_info_v7.board_info_v3.hw_platform;
		board.platform_subtype = board_info_v7.platform_subtype;
		board.platform_version = board_info_v7.platform_version;
		board.pmic_model = board_info_v7.pmic_type;
		board.pmic_die_version = board_info_v7.pmic_version;

	}
	else
	{
		dprintf(CRITICAL, "Unsupported board info format\n");
		ASSERT(0);
	}
}

static void baseband_detect()
{
	unsigned baseband = BASEBAND_MSM;
	unsigned platform_subtype;
	unsigned platform_id;

	platform_id = board.platform;
	platform_subtype = board.platform_subtype;

	/* Check for MDM or APQ baseband variants.  Default to MSM */
	if (platform_subtype == HW_PLATFORM_SUBTYPE_MDM)
		baseband = BASEBAND_MDM;
	else if (platform_id == APQ8060)
		baseband = BASEBAND_APQ;
	else if (platform_id == APQ8064)
		baseband = BASEBAND_APQ;
	else if (platform_id == MPQ8064)
		baseband = BASEBAND_APQ;
	else
		baseband = BASEBAND_MSM;

	board.baseband = baseband;
}

void board_init()
{
	platform_detect();
	target_detect(&board);
	target_baseband_detect(&board);
}

uint32_t board_platform_id(void)
{
	return board.platform;
}

uint32_t board_target_id()
{
	return board.target;
}

uint32_t board_baseband()
{
	return board.baseband;
}

uint32_t board_hardware_id()
{
	return board.platform_hw;
}

uint32_t board_platform_hw(void)
{
	return board.platform_hw;
}

uint32_t board_platform_subtype(void)
{
	return board.platform_subtype;
}

uint32_t board_platform_version(void)
{
	return board.platform_version;
}

uint32_t board_soc_id(void)
{
	return board.soc_id;
}

uint32_t board_soc_version(void)
{
	return board.soc_version;
}

uint32_t board_pmic_model(void)
{
	return board.pmic_model;
}

uint32_t board_pmic_die_version(void)
{
	return board.pmic_die_version;
}
