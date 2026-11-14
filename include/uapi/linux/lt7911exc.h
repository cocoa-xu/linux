/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Lontium LT7911EXC USB-C/DisplayPort to MIPI CSI-2 bridge
 */

#ifndef __UAPI_LT7911EXC_H_
#define __UAPI_LT7911EXC_H_

#include <linux/v4l2-controls.h>

/* Sample rate of the audio the source sends, in Hz; 0 when there is none. */
#define LT7911EXC_CID_AUDIO_SAMPLING_RATE	(V4L2_CID_USER_LT7911EXC_BASE + 0)
/* Whether the source sends audio. */
#define LT7911EXC_CID_AUDIO_PRESENT		(V4L2_CID_USER_LT7911EXC_BASE + 1)

#endif /* __UAPI_LT7911EXC_H_ */
