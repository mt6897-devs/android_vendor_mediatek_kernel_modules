/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 duchampgc16b3front_Sensor_setting.h
 *
 * Project:
 * --------
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _DUCHAMPGC16B3FRONT_SENSOR_SETTING_H
#define _DUCHAMPGC16B3FRONT_SENSOR_SETTING_H

#include "kd_camera_typedef.h"

// NOTE:
// for 2 exp setting,  VCID of LE/SE should be 0x00 and 0x02
// which align 3 exp setting LE/NE/SE 0x00,  0x01,  0x02
// to seamless switch,  VC ID of SE should remain the same
// SONY sensor: VCID of 2nd frame at 0x3070; VCID of 3rd frame at 0x3080
// must be two different value

//init_setting
static u16 duchampgc16b3front_init_setting[] = {
	0x0315, 0xd7,
	0x03a2, 0x0f,
	0x0321, 0x10,
	0x0c0c, 0x33,
	0x0187, 0x40,
	0x0188, 0x5f,
	0x0335, 0x51,
	0x0336, 0x97,
	0x0314, 0x11,
	0x031a, 0x00,
	0x0337, 0x05,
	0x0316, 0x08,
	0x0c0e, 0x40,
	0x0c0d, 0xac,
	0x0334, 0x40,
	0x031c, 0xe0,
	0x0311, 0xf8,
	0x0268, 0x03,
	0x02d1, 0x19,
	0x05a0, 0x0a,
	0x05c3, 0x50,
	0x0217, 0x20,
	0x0074, 0x0a,
	0x00a0, 0x04,
	0x0057, 0x0c,
	0x0358, 0x05,
	0x0059, 0x11,
	0x0084, 0x90,
	0x0087, 0x51,
	0x0c08, 0x19,
	0x02d0, 0x40,
	0x0101, 0x00,
	0x0af0, 0x00,
	0x0c15, 0x05,
	0x0c55, 0x05,
	0x0244, 0x15,
	0x0245, 0x15,
	0x0348, 0x12,
	0x0349, 0x30,
	0x0342, 0x07,
	0x0343, 0x4e,
	0x0219, 0x05,
	0x0e0a, 0x01,
	0x0e0b, 0x01,
	0x0e01, 0x75,
	0x0e03, 0x44,
	0x0e04, 0x44,
	0x0e05, 0x44,
	0x0e06, 0x44,
	0x0e36, 0x06,
	0x0e34, 0xf8,
	0x0e35, 0x34,
	0x0e15, 0x5a,
	0x0e16, 0xaa,
	0x025c, 0xe0,
	0x0c05, 0xbf,
	0x0c09, 0x20,
	0x0c41, 0x0a,
	0x0c42, 0x00,
	0x0c44, 0x00,
	0x0c45, 0xdf,
	0x0e42, 0x0f,
	0x0e44, 0x04,
	0x0e48, 0x00,
	0x0e4f, 0x04,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0xe0,
	0x02db, 0x01,
	0x0b00, 0x0f,
	0x0b01, 0xa2,
	0x0b02, 0x03,
	0x0b03, 0x07,
	0x0b04, 0x11,
	0x0b05, 0x14,
	0x0b06, 0x03,
	0x0b07, 0x07,
	0x0b08, 0xac,
	0x0b09, 0x0d,
	0x0b0a, 0x0c,
	0x0b0b, 0x07,
	0x0b0c, 0x40,
	0x0b0d, 0x34,
	0x0b0e, 0x03,
	0x0b0f, 0x07,
	0x0b10, 0x80,
	0x0b11, 0x1c,
	0x0b12, 0x03,
	0x0b13, 0x07,
	0x0b14, 0x10,
	0x0b15, 0xfe,
	0x0b16, 0x03,
	0x0b17, 0x07,
	0x0b18, 0x00,
	0x0b19, 0xfe,
	0x0b1a, 0x03,
	0x0b1b, 0x07,
	0x0b1c, 0x9f,
	0x0b1d, 0x1c,
	0x0b1e, 0x03,
	0x0b1f, 0x07,
	0x0b20, 0x00,
	0x0b21, 0xfe,
	0x0b22, 0x03,
	0x0b23, 0x07,
	0x0b24, 0x00,
	0x0b25, 0xfe,
	0x0b26, 0x03,
	0x0b27, 0x07,
	0x0b28, 0x80,
	0x0b29, 0x1c,
	0x0b2a, 0x03,
	0x0b2b, 0x07,
	0x0b2c, 0x10,
	0x0b2d, 0xfe,
	0x0b2e, 0x03,
	0x0b2f, 0x07,
	0x0b30, 0x00,
	0x0b31, 0xfe,
	0x0b32, 0x03,
	0x0b33, 0x07,
	0x0b34, 0x9f,
	0x0b35, 0x1c,
	0x0b36, 0x03,
	0x0b37, 0x07,
	0x0b38, 0x46,
	0x0b39, 0x80,
	0x0b3a, 0x01,
	0x0b3b, 0x07,
	0x0b3c, 0x10,
	0x0b3d, 0x84,
	0x0b3e, 0x00,
	0x0b3f, 0x07,
	0x0b40, 0xb8,
	0x0b41, 0x11,
	0x0b42, 0x03,
	0x0b43, 0x07,
	0x0b44, 0x99,
	0x0b45, 0x02,
	0x0b46, 0x01,
	0x0b47, 0x07,
	0x0b48, 0xd9,
	0x0b49, 0x02,
	0x0b4a, 0x01,
	0x0b4b, 0x07,
	0x0b4c, 0x00,
	0x0b4d, 0xfe,
	0x0b4e, 0x03,
	0x0b4f, 0x07,
	0x0b50, 0x06,
	0x0b51, 0x14,
	0x0b52, 0x03,
	0x0b53, 0x07,
	0x0b54, 0x2c,
	0x0b55, 0x0d,
	0x0b56, 0x0c,
	0x0b57, 0x07,
	0x0b58, 0x00,
	0x0b59, 0x34,
	0x0b5a, 0x03,
	0x0b5b, 0x07,
	0x0b5c, 0xe0,
	0x0b5d, 0x1c,
	0x0b5e, 0x03,
	0x0b5f, 0x07,
	0x0b60, 0x90,
	0x0b61, 0x84,
	0x0b62, 0x00,
	0x0b63, 0x07,
	0x0b64, 0x06,
	0x0b65, 0x80,
	0x0b66, 0x01,
	0x0b67, 0x07,
	0x0b68, 0x07,
	0x0b69, 0xa2,
	0x0b6a, 0x03,
	0x0b6b, 0x07,
	0x0aab, 0x01,
	0x0af0, 0x02,
	0x0aa8, 0xb0,
	0x0aa9, 0x92,
	0x0aaa, 0x1b,
	0x0264, 0x00,
	0x0265, 0x04,
	0x0266, 0x1e,
	0x0267, 0x10,
	0x0041, 0x30,
	0x0043, 0x00,
	0x0044, 0x01,
	0x005b, 0x02,
	0x0047, 0xf0,
	0x0048, 0x0f,
	0x004b, 0x0f,
	0x004c, 0x00,
	0x024a, 0x02,
	0x0249, 0x00,
	0x024f, 0x0e,
	0x024e, 0x80,
	0x0c12, 0xe6,
	0x0c52, 0xe6,
	0x0c10, 0x20,
	0x0c11, 0x58,
	0x0c50, 0x20,
	0x0c51, 0x58,
	0x0460, 0x08,
	0x0462, 0x06,
	0x0464, 0x04,
	0x0466, 0x02,
	0x0468, 0x10,
	0x046a, 0x0e,
	0x046c, 0x0e,
	0x046e, 0x0c,
	0x0461, 0x03,
	0x0463, 0x03,
	0x0465, 0x03,
	0x0467, 0x03,
	0x0469, 0x04,
	0x046b, 0x04,
	0x046d, 0x04,
	0x046f, 0x04,
	0x0470, 0x04,
	0x0472, 0x08,
	0x0474, 0x0c,
	0x0476, 0x10,
	0x0478, 0x06,
	0x047a, 0x06,
	0x047c, 0x08,
	0x047e, 0x08,
	0x0471, 0x04,
	0x0473, 0x04,
	0x0475, 0x04,
	0x0477, 0x04,
	0x0479, 0x03,
	0x047b, 0x03,
	0x047d, 0x03,
	0x047f, 0x03,
};

// mode 0: 2304x1728@30fps, normal preview + no pd
static u16 duchampgc16b3front_preview_setting[] = {
	0x0315, 0xd3,
	0x03a2, 0x0f,
	0x0321, 0x10,
	0x0c0c, 0x33,
	0x0187, 0x40,
	0x0188, 0x5f,
	0x0335, 0x59,
	0x0336, 0x97,
	0x0314, 0x11,
	0x031a, 0x01,
	0x0337, 0x05,
	0x0316, 0x08,
	0x0c0e, 0x41,
	0x0c0d, 0xac,
	0x0334, 0x40,
	0x031c, 0xe0,
	0x0311, 0xf8,
	0x0268, 0x03,
	0x0218, 0x01,
	0x0241, 0xd4,
	0x0346, 0x00,
	0x0347, 0x04,
	0x034a, 0x0d,
	0x034b, 0xb0,
	0x0342, 0x07,
	0x0343, 0x2c,
	0x0226, 0x00,
	0x0227, 0x40,
	0x0202, 0x06,
	0x0203, 0x8a,
	0x0340, 0x07,
	0x0341, 0x28,
	0x0e24, 0x02,
	0x0e25, 0x02,
	0x0e2c, 0x08,
	0x0e2d, 0x0c,
	0x0e37, 0x41,
	0x0e38, 0x41,
	0x0e17, 0x36,
	0x0e18, 0x39,
	0x0e19, 0x60,
	0x0e1a, 0x62,
	0x0e49, 0x3a,
	0x0e2b, 0x6c,
	0x0e0c, 0x28,
	0x0e28, 0x28,
	0x0210, 0xa3,
	0x02b5, 0x84,
	0x02b6, 0x72,
	0x02b7, 0x0e,
	0x02b8, 0x05,
	0x0c07, 0xec,
	0x0c46, 0xfe,
	0x0c47, 0x02,
	0x0e43, 0x00,
	0x0e45, 0x04,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0xe0,
	0x0360, 0x01,
	0x0360, 0x00,
	0x0a67, 0x80,
	0x0313, 0x00,
	0x0ace, 0x08,
	0x0a53, 0x04,
	0x0a65, 0x05,
	0x0a68, 0x11,
	0x0a58, 0x00,
	0x00a4, 0x00,
	0x00a5, 0x01,
	0x00a2, 0x00,
	0x00a3, 0x00,
	0x00ab, 0x00,
	0x00ac, 0x00,
	0x00a7, 0x0d,
	0x00a8, 0xb0,
	0x00a9, 0x12,
	0x00aa, 0x30,
	0x0a85, 0x1e,
	0x0a86, 0xa8,
	0x0a8a, 0x00,
	0x0a8b, 0xe0,
	0x0a8c, 0x1e,
	0x0a8d, 0x10,
	0x0a90, 0x08,
	0x0a91, 0x1c,
	0x0a92, 0x78,
	0x0a71, 0xd2,
	0x0a72, 0x12,
	0x0a73, 0x60,
	0x0a75, 0x41,
	0x0a70, 0x87,
	0x0313, 0x80,
	0x0042, 0x00,
	0x0056, 0x00,
	0x0488, 0x06,
	0x048a, 0x06,
	0x048c, 0x06,
	0x048e, 0x06,
	0x05a0, 0x82,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x0597, 0x6b,
	0x059a, 0x00,
	0x059b, 0x00,
	0x059c, 0x01,
	0x05a3, 0x0a,
	0x05a4, 0x08,
	0x05ab, 0x0a,
	0x05ae, 0x00,
	0x0108, 0x48,
	0x010b, 0x12,
	0x01c1, 0x95,
	0x01c2, 0x00,
	0x0800, 0x05,
	0x0801, 0x06,
	0x0802, 0x0a,
	0x0803, 0x0d,
	0x0804, 0x12,
	0x0805, 0x17,
	0x0806, 0x22,
	0x0807, 0x2e,
	0x0808, 0x5a,
	0x0809, 0x0e,
	0x080a, 0x32,
	0x080b, 0x0e,
	0x080c, 0x33,
	0x080d, 0x02,
	0x080e, 0xb8,
	0x080f, 0x03,
	0x0810, 0x1d,
	0x0811, 0x00,
	0x0812, 0xc0,
	0x0813, 0x03,
	0x0814, 0x1d,
	0x0815, 0x03,
	0x0816, 0x1e,
	0x0817, 0x03,
	0x0818, 0x1e,
	0x0819, 0x02,
	0x081a, 0x08,
	0x081b, 0x3e,
	0x081c, 0x02,
	0x081d, 0x00,
	0x081e, 0x00,
	0x081f, 0x01,
	0x0820, 0x01,
	0x0821, 0x02,
	0x0822, 0x06,
	0x0823, 0x3e,
	0x0824, 0x02,
	0x0825, 0x00,
	0x0826, 0x00,
	0x0827, 0x01,
	0x0828, 0x01,
	0x0829, 0x02,
	0x082a, 0x02,
	0x082b, 0x3e,
	0x082c, 0x02,
	0x082d, 0x00,
	0x082e, 0x00,
	0x082f, 0x01,
	0x0830, 0x01,
	0x0831, 0x01,
	0x0832, 0x1c,
	0x0833, 0x3e,
	0x0834, 0x02,
	0x0835, 0x00,
	0x0836, 0x00,
	0x0837, 0x01,
	0x0838, 0x01,
	0x0839, 0x01,
	0x083a, 0x16,
	0x083b, 0x3e,
	0x083c, 0x02,
	0x083d, 0x00,
	0x083e, 0x00,
	0x083f, 0x01,
	0x0840, 0x01,
	0x0841, 0x01,
	0x0842, 0x10,
	0x0843, 0x3e,
	0x0844, 0x02,
	0x0845, 0x00,
	0x0846, 0x00,
	0x0847, 0x01,
	0x0848, 0x01,
	0x0849, 0x01,
	0x084a, 0x08,
	0x084b, 0x3e,
	0x084c, 0x02,
	0x084d, 0x00,
	0x084e, 0x00,
	0x084f, 0x01,
	0x0850, 0x01,
	0x0851, 0x00,
	0x0852, 0x1e,
	0x0853, 0x3e,
	0x0854, 0x02,
	0x0855, 0x00,
	0x0856, 0x00,
	0x0857, 0x01,
	0x0858, 0x01,
	0x0859, 0x00,
	0x085a, 0x14,
	0x085b, 0x3e,
	0x085c, 0x02,
	0x085d, 0x02,
	0x085e, 0x00,
	0x085f, 0x01,
	0x0860, 0x01,
	0x0861, 0x00,
	0x0862, 0x0c,
	0x0863, 0x36,
	0x0864, 0x02,
	0x0865, 0x02,
	0x0866, 0x00,
	0x0867, 0x01,
	0x0868, 0x01,
	0x0869, 0x00,
	0x086a, 0x00,
	0x086b, 0x01,
	0x086c, 0x00,
	0x086d, 0x01,
	0x086e, 0x00,
	0x086f, 0x00,
	0x0870, 0x01,
	0x0871, 0x01,
	0x0872, 0x62,
	0x0873, 0x00,
	0x0874, 0x02,
	0x0875, 0x01,
	0x0876, 0xf8,
	0x0877, 0x00,
	0x0878, 0x03,
	0x0879, 0x02,
	0x087a, 0xc0,
	0x087b, 0x00,
	0x087c, 0x04,
	0x087d, 0x03,
	0x087e, 0xeb,
	0x087f, 0x00,
	0x0880, 0x05,
	0x0881, 0x05,
	0x0882, 0x7a,
	0x0883, 0x00,
	0x0884, 0x06,
	0x0885, 0x07,
	0x0886, 0xe0,
	0x0887, 0x10,
	0x0888, 0x05,
	0x0889, 0x0b,
	0x088a, 0x02,
	0x088b, 0x10,
	0x088c, 0x06,
	0x088d, 0x0f,
	0x088e, 0x92,
	0x088f, 0x14,
	0x0890, 0xb6,
	0x0891, 0x1f,
	0x0892, 0xab,
	0x0893, 0x1a,
	0x0894, 0x66,
	0x0895, 0x01,
	0x0896, 0x46,
	0x0897, 0x02,
	0x0898, 0x01,
	0x0899, 0x01,
	0x089a, 0x01,
	0x089b, 0x03,
	0x089c, 0x4c,
	0x089d, 0x04,
	0x089e, 0xff,
	0x089f, 0xff,
	0x08a0, 0x99,
	0x08a1, 0x02,
	0x08a2, 0x02,
	0x08a3, 0x04,
	0x08a4, 0x02,
	0x08a5, 0x0e,
	0x08a6, 0x02,
	0x08a7, 0x03,
	0x08a8, 0x40,
	0x08a9, 0x04,
	0x08aa, 0xff,
	0x08ab, 0xff,
	0x08ac, 0x00,
	0x05ac, 0x01,
	0x0207, 0xc4,
	0x05a0, 0xc2,
	0x01c0, 0x01,
	0x0096, 0x81,
	0x0097, 0x08,
	0x0098, 0x87,
	0x0204, 0x04,
	0x0205, 0x00,
	0x0208, 0x01,
	0x0209, 0x6f,
	0x0351, 0x00,
	0x0352, 0x0c,
	0x0353, 0x00,
	0x0354, 0x0c,
	0x034c, 0x09,
	0x034d, 0x00,
	0x034e, 0x06,
	0x034f, 0xc0,
	0x0180, 0x46,
	0x0181, 0xf0,
	0x0185, 0x01,
	0x0103, 0x10,
	0x0106, 0x39,
	0x0114, 0x03,
	0x0115, 0x20,
	0x0121, 0x02,
	0x0122, 0x03,
	0x0123, 0x0a,
	0x0124, 0x00,
	0x0125, 0x18,
	0x0126, 0x04,
	0x0128, 0xf0,
	0x0129, 0x03,
	0x012a, 0x02,
	0x012b, 0x05,
	0x0a70, 0x11,
	0x0313, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x0a70, 0x00,
	0x0070, 0x05,
	0x0089, 0x03,
	0x009b, 0x40,
	0x00a4, 0x80,
	0x00a0, 0x05,
	0x00a6, 0x07,
	0x0080, 0xd2,
	0x00c1, 0x80,
	0x00c2, 0x11,
	0x024d, 0x01,
	0x0084, 0x10,
	0x0268, 0x00,
	0x031c, 0x9f,
};

// mode 1: same as preview mode 
static u16 duchampgc16b3front_capture_setting[] = {};

// mode 2: 2304x1296@30fps, noraml video + no pd
static u16 duchampgc16b3front_normal_video_setting[] = {
	0x0315, 0xd3,
	0x03a2, 0x0f,
	0x0321, 0x10,
	0x0c0c, 0x33,
	0x0187, 0x40,
	0x0188, 0x5f,
	0x0335, 0x59,
	0x0336, 0x97,
	0x0314, 0x11,
	0x031a, 0x01,
	0x0337, 0x05,
	0x0316, 0x08,
	0x0c0e, 0x41,
	0x0c0d, 0xac,
	0x0334, 0x40,
	0x031c, 0xe0,
	0x0311, 0xf8,
	0x0268, 0x03,
	0x0218, 0x01,
	0x0241, 0xd4,
	0x0346, 0x00,
	0x0347, 0x04,
	0x034a, 0x0d,
	0x034b, 0xb0,
	0x0342, 0x07,
	0x0343, 0x2c,
	0x0226, 0x00,
	0x0227, 0x40,
	0x0202, 0x06,
	0x0203, 0x8a,
	0x0340, 0x07,
	0x0341, 0x28,
	0x0e24, 0x02,
	0x0e25, 0x02,
	0x0e2c, 0x08,
	0x0e2d, 0x0c,
	0x0e37, 0x41,
	0x0e38, 0x41,
	0x0e17, 0x36,
	0x0e18, 0x39,
	0x0e19, 0x60,
	0x0e1a, 0x62,
	0x0e49, 0x3a,
	0x0e2b, 0x6c,
	0x0e0c, 0x28,
	0x0e28, 0x28,
	0x0210, 0xa3,
	0x02b5, 0x84,
	0x02b6, 0x72,
	0x02b7, 0x0e,
	0x02b8, 0x05,
	0x0c07, 0xec,
	0x0c46, 0xfe,
	0x0c47, 0x02,
	0x0e43, 0x00,
	0x0e45, 0x04,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0xe0,
	0x0360, 0x01,
	0x0360, 0x00,
	0x0a67, 0x80,
	0x0313, 0x00,
	0x0ace, 0x08,
	0x0a53, 0x04,
	0x0a65, 0x05,
	0x0a68, 0x11,
	0x0a58, 0x00,
	0x00a4, 0x00,
	0x00a5, 0x01,
	0x00a2, 0x00,
	0x00a3, 0x00,
	0x00ab, 0x00,
	0x00ac, 0x00,
	0x00a7, 0x0d,
	0x00a8, 0xb0,
	0x00a9, 0x12,
	0x00aa, 0x30,
	0x0a85, 0x1e,
	0x0a86, 0xa8,
	0x0a8a, 0x00,
	0x0a8b, 0xe0,
	0x0a8c, 0x1e,
	0x0a8d, 0x10,
	0x0a90, 0x08,
	0x0a91, 0x1c,
	0x0a92, 0x78,
	0x0a71, 0xd2,
	0x0a72, 0x12,
	0x0a73, 0x60,
	0x0a75, 0x41,
	0x0a70, 0x87,
	0x0313, 0x80,
	0x0042, 0x00,
	0x0056, 0x00,
	0x0488, 0x06,
	0x048a, 0x06,
	0x048c, 0x06,
	0x048e, 0x06,
	0x05a0, 0x82,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x0597, 0x6b,
	0x059a, 0x00,
	0x059b, 0x00,
	0x059c, 0x01,
	0x05a3, 0x0a,
	0x05a4, 0x08,
	0x05ab, 0x0a,
	0x05ae, 0x00,
	0x0108, 0x48,
	0x010b, 0x12,
	0x01c1, 0x95,
	0x01c2, 0x00,
	0x0800, 0x05,
	0x0801, 0x06,
	0x0802, 0x0a,
	0x0803, 0x0d,
	0x0804, 0x12,
	0x0805, 0x17,
	0x0806, 0x22,
	0x0807, 0x2e,
	0x0808, 0x5a,
	0x0809, 0x0e,
	0x080a, 0x32,
	0x080b, 0x0e,
	0x080c, 0x33,
	0x080d, 0x02,
	0x080e, 0xb8,
	0x080f, 0x03,
	0x0810, 0x1d,
	0x0811, 0x00,
	0x0812, 0xc0,
	0x0813, 0x03,
	0x0814, 0x1d,
	0x0815, 0x03,
	0x0816, 0x1e,
	0x0817, 0x03,
	0x0818, 0x1e,
	0x0819, 0x02,
	0x081a, 0x08,
	0x081b, 0x3e,
	0x081c, 0x02,
	0x081d, 0x00,
	0x081e, 0x00,
	0x081f, 0x01,
	0x0820, 0x01,
	0x0821, 0x02,
	0x0822, 0x06,
	0x0823, 0x3e,
	0x0824, 0x02,
	0x0825, 0x00,
	0x0826, 0x00,
	0x0827, 0x01,
	0x0828, 0x01,
	0x0829, 0x02,
	0x082a, 0x02,
	0x082b, 0x3e,
	0x082c, 0x02,
	0x082d, 0x00,
	0x082e, 0x00,
	0x082f, 0x01,
	0x0830, 0x01,
	0x0831, 0x01,
	0x0832, 0x1c,
	0x0833, 0x3e,
	0x0834, 0x02,
	0x0835, 0x00,
	0x0836, 0x00,
	0x0837, 0x01,
	0x0838, 0x01,
	0x0839, 0x01,
	0x083a, 0x16,
	0x083b, 0x3e,
	0x083c, 0x02,
	0x083d, 0x00,
	0x083e, 0x00,
	0x083f, 0x01,
	0x0840, 0x01,
	0x0841, 0x01,
	0x0842, 0x10,
	0x0843, 0x3e,
	0x0844, 0x02,
	0x0845, 0x00,
	0x0846, 0x00,
	0x0847, 0x01,
	0x0848, 0x01,
	0x0849, 0x01,
	0x084a, 0x08,
	0x084b, 0x3e,
	0x084c, 0x02,
	0x084d, 0x00,
	0x084e, 0x00,
	0x084f, 0x01,
	0x0850, 0x01,
	0x0851, 0x00,
	0x0852, 0x1e,
	0x0853, 0x3e,
	0x0854, 0x02,
	0x0855, 0x00,
	0x0856, 0x00,
	0x0857, 0x01,
	0x0858, 0x01,
	0x0859, 0x00,
	0x085a, 0x14,
	0x085b, 0x3e,
	0x085c, 0x02,
	0x085d, 0x02,
	0x085e, 0x00,
	0x085f, 0x01,
	0x0860, 0x01,
	0x0861, 0x00,
	0x0862, 0x0c,
	0x0863, 0x36,
	0x0864, 0x02,
	0x0865, 0x02,
	0x0866, 0x00,
	0x0867, 0x01,
	0x0868, 0x01,
	0x0869, 0x00,
	0x086a, 0x00,
	0x086b, 0x01,
	0x086c, 0x00,
	0x086d, 0x01,
	0x086e, 0x00,
	0x086f, 0x00,
	0x0870, 0x01,
	0x0871, 0x01,
	0x0872, 0x62,
	0x0873, 0x00,
	0x0874, 0x02,
	0x0875, 0x01,
	0x0876, 0xf8,
	0x0877, 0x00,
	0x0878, 0x03,
	0x0879, 0x02,
	0x087a, 0xc0,
	0x087b, 0x00,
	0x087c, 0x04,
	0x087d, 0x03,
	0x087e, 0xeb,
	0x087f, 0x00,
	0x0880, 0x05,
	0x0881, 0x05,
	0x0882, 0x7a,
	0x0883, 0x00,
	0x0884, 0x06,
	0x0885, 0x07,
	0x0886, 0xe0,
	0x0887, 0x10,
	0x0888, 0x05,
	0x0889, 0x0b,
	0x088a, 0x02,
	0x088b, 0x10,
	0x088c, 0x06,
	0x088d, 0x0f,
	0x088e, 0x92,
	0x088f, 0x14,
	0x0890, 0xb6,
	0x0891, 0x1f,
	0x0892, 0xab,
	0x0893, 0x1a,
	0x0894, 0x66,
	0x0895, 0x01,
	0x0896, 0x46,
	0x0897, 0x02,
	0x0898, 0x01,
	0x0899, 0x01,
	0x089a, 0x01,
	0x089b, 0x03,
	0x089c, 0x4c,
	0x089d, 0x04,
	0x089e, 0xff,
	0x089f, 0xff,
	0x08a0, 0x99,
	0x08a1, 0x02,
	0x08a2, 0x02,
	0x08a3, 0x04,
	0x08a4, 0x02,
	0x08a5, 0x0e,
	0x08a6, 0x02,
	0x08a7, 0x03,
	0x08a8, 0x40,
	0x08a9, 0x04,
	0x08aa, 0xff,
	0x08ab, 0xff,
	0x08ac, 0x00,
	0x05ac, 0x01,
	0x0207, 0xc4,
	0x05a0, 0xc2,
	0x01c0, 0x01,
	0x0096, 0x81,
	0x0097, 0x08,
	0x0098, 0x87,
	0x0204, 0x04,
	0x0205, 0x00,
	0x0208, 0x01,
	0x0209, 0x6f,
	0x0351, 0x00,
	0x0352, 0xe4,
	0x0353, 0x00,
	0x0354, 0x0c,
	0x034c, 0x09,
	0x034d, 0x00,
	0x034e, 0x05,
	0x034f, 0x10,
	0x0180, 0x46,
	0x0181, 0xf0,
	0x0185, 0x01,
	0x0103, 0x10,
	0x0106, 0x39,
	0x0114, 0x03,
	0x0115, 0x20,
	0x0121, 0x02,
	0x0122, 0x03,
	0x0123, 0x0a,
	0x0124, 0x00,
	0x0125, 0x18,
	0x0126, 0x04,
	0x0128, 0xf0,
	0x0129, 0x03,
	0x012a, 0x02,
	0x012b, 0x05,
	0x0a70, 0x11,
	0x0313, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x0a70, 0x00,
	0x0070, 0x05,
	0x0089, 0x03,
	0x009b, 0x40,
	0x00a4, 0x80,
	0x00a0, 0x05,
	0x00a6, 0x07,
	0x0080, 0xd2,
	0x00c1, 0x80,
	0x00c2, 0x11,
	0x024d, 0x01,
	0x0084, 0x10,
	0x0268, 0x00,
	0x031c, 0x9f,
};

// mode 3: 1920x1080@120fps, SMVR + pd
static u16 duchampgc16b3front_hs_video_setting[] = {
	0x0315, 0xd7,
	0x03a2, 0x0f,
	0x0321, 0x10,
	0x0c0c, 0x33,
	0x0187, 0x40,
	0x0188, 0x5f,
	0x0335, 0x55,
	0x0336, 0x97,
	0x0314, 0x11,
	0x031a, 0x00,
	0x0337, 0x05,
	0x0316, 0x08,
	0x0c0e, 0x40,
	0x0c0d, 0xac,
	0x0334, 0x40,
	0x031c, 0xe0,
	0x0311, 0xf8,
	0x0268, 0x03,
	0x0218, 0x01,
	0x0241, 0xd4,
	0x0346, 0x04,
	0x0347, 0x04,
	0x034a, 0x05,
	0x034b, 0xb0,
	0x0342, 0x07,
	0x0343, 0x72,
	0x0226, 0x00,
	0x0227, 0x90,
	0x0202, 0x02,
	0x0203, 0x18,
	0x0340, 0x03,
	0x0341, 0x78,
	0x0e24, 0x02,
	0x0e25, 0x02,
	0x0e2c, 0x08,
	0x0e2d, 0x0c,
	0x0e37, 0x35,
	0x0e38, 0x35,
	0x0e17, 0x48,
	0x0e18, 0x4b,
	0x0e19, 0x56,
	0x0e1a, 0x58,
	0x0e49, 0x4c,
	0x0e2b, 0x60,
	0x0e0c, 0x40,
	0x0e28, 0x40,
	0x0210, 0xa3,
	0x02b5, 0x84,
	0x02b6, 0x72,
	0x02b7, 0x0e,
	0x02b8, 0x05,
	0x0c07, 0xec,
	0x0c46, 0xfe,
	0x0c47, 0x02,
	0x0e43, 0x00,
	0x0e45, 0x04,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0xe0,
	0x0360, 0x01,
	0x0360, 0x00,
	0x0a67, 0x80,
	0x0313, 0x00,
	0x0ace, 0x08,
	0x0a53, 0x04,
	0x0a65, 0x05,
	0x0a68, 0x11,
	0x0a58, 0x00,
	0x00a4, 0x00,
	0x00a5, 0x01,
	0x00a2, 0x00,
	0x00a3, 0x04,
	0x00ab, 0x00,
	0x00ac, 0x00,
	0x00a7, 0x0d,
	0x00a8, 0xb0,
	0x00a9, 0x12,
	0x00aa, 0x30,
	0x0a85, 0x1e,
	0x0a86, 0xa8,
	0x0a8a, 0x00,
	0x0a8b, 0xe0,
	0x0a8c, 0x1e,
	0x0a8d, 0x10,
	0x0a90, 0x08,
	0x0a91, 0x1c,
	0x0a92, 0x78,
	0x0a71, 0xd2,
	0x0a72, 0x12,
	0x0a73, 0x60,
	0x0a75, 0x41,
	0x0a70, 0x87,
	0x0313, 0x80,
	0x0042, 0x00,
	0x0056, 0x00,
	0x0488, 0x06,
	0x048a, 0x06,
	0x048c, 0x06,
	0x048e, 0x06,
	0x05a0, 0x82,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x0597, 0x6b,
	0x059a, 0x00,
	0x059b, 0x00,
	0x059c, 0x01,
	0x05a3, 0x0a,
	0x05a4, 0x08,
	0x05ab, 0x0a,
	0x05ae, 0x00,
	0x0108, 0x48,
	0x010b, 0x12,
	0x01c1, 0x95,
	0x01c2, 0x00,
	0x0800, 0x05,
	0x0801, 0x06,
	0x0802, 0x0a,
	0x0803, 0x0d,
	0x0804, 0x12,
	0x0805, 0x17,
	0x0806, 0x22,
	0x0807, 0x2e,
	0x0808, 0x5a,
	0x0809, 0x0e,
	0x080a, 0x32,
	0x080b, 0x0e,
	0x080c, 0x33,
	0x080d, 0x02,
	0x080e, 0xb8,
	0x080f, 0x03,
	0x0810, 0x1d,
	0x0811, 0x00,
	0x0812, 0xc0,
	0x0813, 0x03,
	0x0814, 0x1d,
	0x0815, 0x03,
	0x0816, 0x1e,
	0x0817, 0x03,
	0x0818, 0x1e,
	0x0819, 0x01,
	0x081a, 0x18,
	0x081b, 0x06,
	0x081c, 0x02,
	0x081d, 0x00,
	0x081e, 0x00,
	0x081f, 0x01,
	0x0820, 0x01,
	0x0821, 0x01,
	0x0822, 0x16,
	0x0823, 0x06,
	0x0824, 0x02,
	0x0825, 0x00,
	0x0826, 0x00,
	0x0827, 0x01,
	0x0828, 0x01,
	0x0829, 0x01,
	0x082a, 0x12,
	0x082b, 0x06,
	0x082c, 0x02,
	0x082d, 0x00,
	0x082e, 0x00,
	0x082f, 0x01,
	0x0830, 0x01,
	0x0831, 0x01,
	0x0832, 0x10,
	0x0833, 0x06,
	0x0834, 0x02,
	0x0835, 0x00,
	0x0836, 0x00,
	0x0837, 0x01,
	0x0838, 0x01,
	0x0839, 0x01,
	0x083a, 0x0e,
	0x083b, 0x06,
	0x083c, 0x02,
	0x083d, 0x00,
	0x083e, 0x00,
	0x083f, 0x01,
	0x0840, 0x01,
	0x0841, 0x01,
	0x0842, 0x0a,
	0x0843, 0x06,
	0x0844, 0x02,
	0x0845, 0x00,
	0x0846, 0x00,
	0x0847, 0x01,
	0x0848, 0x01,
	0x0849, 0x01,
	0x084a, 0x02,
	0x084b, 0x06,
	0x084c, 0x02,
	0x084d, 0x00,
	0x084e, 0x00,
	0x084f, 0x01,
	0x0850, 0x01,
	0x0851, 0x00,
	0x0852, 0x1c,
	0x0853, 0x06,
	0x0854, 0x02,
	0x0855, 0x00,
	0x0856, 0x00,
	0x0857, 0x01,
	0x0858, 0x01,
	0x0859, 0x00,
	0x085a, 0x16,
	0x085b, 0x06,
	0x085c, 0x02,
	0x085d, 0x02,
	0x085e, 0x00,
	0x085f, 0x01,
	0x0860, 0x01,
	0x0861, 0x00,
	0x0862, 0x0a,
	0x0863, 0x06,
	0x0864, 0x02,
	0x0865, 0x02,
	0x0866, 0x00,
	0x0867, 0x01,
	0x0868, 0x01,
	0x0869, 0x00,
	0x086a, 0x00,
	0x086b, 0x01,
	0x086c, 0x00,
	0x086d, 0x01,
	0x086e, 0x00,
	0x086f, 0x00,
	0x0870, 0x01,
	0x0871, 0x01,
	0x0872, 0x61,
	0x0873, 0x00,
	0x0874, 0x02,
	0x0875, 0x01,
	0x0876, 0xf4,
	0x0877, 0x00,
	0x0878, 0x03,
	0x0879, 0x02,
	0x087a, 0xbb,
	0x087b, 0x00,
	0x087c, 0x04,
	0x087d, 0x03,
	0x087e, 0xe5,
	0x087f, 0x00,
	0x0880, 0x05,
	0x0881, 0x05,
	0x0882, 0x73,
	0x0883, 0x00,
	0x0884, 0x06,
	0x0885, 0x07,
	0x0886, 0xd6,
	0x0887, 0x10,
	0x0888, 0x05,
	0x0889, 0x0a,
	0x088a, 0xfb,
	0x088b, 0x10,
	0x088c, 0x06,
	0x088d, 0x0f,
	0x088e, 0x89,
	0x088f, 0x14,
	0x0890, 0xb6,
	0x0891, 0x1f,
	0x0892, 0x90,
	0x0893, 0x1a,
	0x0894, 0x66,
	0x0895, 0x01,
	0x0896, 0x46,
	0x0897, 0x02,
	0x0898, 0x01,
	0x0899, 0x01,
	0x089a, 0x01,
	0x089b, 0x03,
	0x089c, 0x4c,
	0x089d, 0x04,
	0x089e, 0xff,
	0x089f, 0xff,
	0x08a0, 0x99,
	0x08a1, 0x02,
	0x08a2, 0x02,
	0x08a3, 0x04,
	0x08a4, 0x02,
	0x08a5, 0x0e,
	0x08a6, 0x02,
	0x08a7, 0x03,
	0x08a8, 0x40,
	0x08a9, 0x04,
	0x08aa, 0xff,
	0x08ab, 0xff,
	0x08ac, 0x00,
	0x05ac, 0x01,
	0x0207, 0xc4,
	0x05a0, 0xc2,
	0x01c0, 0x01,
	0x0096, 0x81,
	0x0097, 0x08,
	0x0098, 0x87,
	0x0204, 0x04,
	0x0205, 0x00,
	0x0208, 0x01,
	0x0209, 0x6f,
	0x0351, 0x00,
	0x0352, 0x04,
	0x0353, 0x02,
	0x0354, 0x0c,
	0x034c, 0x05,
	0x034d, 0x00,
	0x034e, 0x02,
	0x034f, 0xd0,
	0x0180, 0x46,
	0x0181, 0xf0,
	0x0185, 0x01,
	0x0103, 0x10,
	0x0106, 0x39,
	0x0114, 0x03,
	0x0115, 0x20,
	0x0121, 0x05,
	0x0122, 0x07,
	0x0123, 0x17,
	0x0124, 0x00,
	0x0125, 0x16,
	0x0126, 0x08,
	0x0128, 0xf0,
	0x0129, 0x07,
	0x012a, 0x09,
	0x012b, 0x09,
	0x0a70, 0x11,
	0x0313, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x0a70, 0x00,
	0x0070, 0x05,
	0x0089, 0x03,
	0x009b, 0x40,
	0x00a4, 0x80,
	0x00a0, 0x05,
	0x00a6, 0x07,
	0x0080, 0xd2,
	0x00c0, 0x02,
	0x00c1, 0x80,
	0x00c2, 0x11,
	0x024d, 0x01,
	0x0084, 0x10,
	0x0268, 0x00,
	0x031c, 0x9f,
};

// mode 4: same as preview mode 
static u16 duchampgc16b3front_slim_video_setting[] = {};

// mode 5: 2304x1296@60fps, VHDR(m-stream)/ 60fps video + no pd
static u16 duchampgc16b3front_custom1_setting[] = {
	0x0315, 0xd7,
	0x03a2, 0x0f,
	0x0321, 0x10,
	0x0c0c, 0x33,
	0x0187, 0x40,
	0x0188, 0x5f,
	0x0335, 0x55,
	0x0336, 0x97,
	0x0314, 0x11,
	0x031a, 0x00,
	0x0337, 0x05,
	0x0316, 0x08,
	0x0c0e, 0x40,
	0x0c0d, 0xac,
	0x0334, 0x40,
	0x031c, 0xe0,
	0x0311, 0xf8,
	0x0268, 0x03,
	0x0218, 0x01,
	0x0241, 0xd4,
	0x0346, 0x00,
	0x0347, 0x04,
	0x034a, 0x0d,
	0x034b, 0xb0,
	0x0342, 0x07,
	0x0343, 0x2c,
	0x0226, 0x00,
	0x0227, 0x40,
	0x0202, 0x06,
	0x0203, 0x48,
	0x0340, 0x07,
	0x0341, 0x40,
	0x0e24, 0x02,
	0x0e25, 0x02,
	0x0e2c, 0x08,
	0x0e2d, 0x0c,
	0x0e37, 0x35,
	0x0e38, 0x35,
	0x0e17, 0x48,
	0x0e18, 0x4b,
	0x0e19, 0x52,
	0x0e1a, 0x54,
	0x0e49, 0x4c,
	0x0e2b, 0x58,
	0x0e0c, 0x38,
	0x0e28, 0x38,
	0x0210, 0xa3,
	0x02b5, 0x84,
	0x02b6, 0x72,
	0x02b7, 0x0e,
	0x02b8, 0x05,
	0x0c07, 0xec,
	0x0c46, 0xfe,
	0x0c47, 0x02,
	0x0e43, 0x00,
	0x0e45, 0x04,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0x80,
	0x03fe, 0x10,
	0x03fe, 0x00,
	0x031c, 0x9f,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x031c, 0xe0,
	0x0360, 0x01,
	0x0360, 0x00,
	0x0a67, 0x80,
	0x0313, 0x00,
	0x0ace, 0x08,
	0x0a53, 0x04,
	0x0a65, 0x05,
	0x0a68, 0x11,
	0x0a58, 0x00,
	0x00a4, 0x00,
	0x00a5, 0x01,
	0x00a2, 0x00,
	0x00a3, 0x00,
	0x00ab, 0x00,
	0x00ac, 0x00,
	0x00a7, 0x0d,
	0x00a8, 0xb0,
	0x00a9, 0x12,
	0x00aa, 0x30,
	0x0a85, 0x1e,
	0x0a86, 0xa8,
	0x0a8a, 0x00,
	0x0a8b, 0xe0,
	0x0a8c, 0x1e,
	0x0a8d, 0x10,
	0x0a90, 0x08,
	0x0a91, 0x1c,
	0x0a92, 0x78,
	0x0a71, 0xd2,
	0x0a72, 0x12,
	0x0a73, 0x60,
	0x0a75, 0x41,
	0x0a70, 0x87,
	0x0313, 0x80,
	0x0042, 0x00,
	0x0056, 0x00,
	0x0488, 0x06,
	0x048a, 0x06,
	0x048c, 0x06,
	0x048e, 0x06,
	0x05a0, 0x82,
	0x05ac, 0x00,
	0x05ad, 0x01,
	0x0597, 0x6b,
	0x059a, 0x00,
	0x059b, 0x00,
	0x059c, 0x01,
	0x05a3, 0x0a,
	0x05a4, 0x08,
	0x05ab, 0x0a,
	0x05ae, 0x00,
	0x0108, 0x48,
	0x010b, 0x12,
	0x01c1, 0x95,
	0x01c2, 0x00,
	0x0800, 0x05,
	0x0801, 0x06,
	0x0802, 0x0a,
	0x0803, 0x0d,
	0x0804, 0x12,
	0x0805, 0x17,
	0x0806, 0x22,
	0x0807, 0x2e,
	0x0808, 0x5a,
	0x0809, 0x0e,
	0x080a, 0x32,
	0x080b, 0x0e,
	0x080c, 0x33,
	0x080d, 0x02,
	0x080e, 0xb8,
	0x080f, 0x03,
	0x0810, 0x1d,
	0x0811, 0x00,
	0x0812, 0xc0,
	0x0813, 0x03,
	0x0814, 0x1d,
	0x0815, 0x03,
	0x0816, 0x1e,
	0x0817, 0x03,
	0x0818, 0x1e,
	0x0819, 0x01,
	0x081a, 0x18,
	0x081b, 0x06,
	0x081c, 0x02,
	0x081d, 0x00,
	0x081e, 0x00,
	0x081f, 0x01,
	0x0820, 0x01,
	0x0821, 0x01,
	0x0822, 0x16,
	0x0823, 0x06,
	0x0824, 0x02,
	0x0825, 0x00,
	0x0826, 0x00,
	0x0827, 0x01,
	0x0828, 0x01,
	0x0829, 0x01,
	0x082a, 0x12,
	0x082b, 0x06,
	0x082c, 0x02,
	0x082d, 0x00,
	0x082e, 0x00,
	0x082f, 0x01,
	0x0830, 0x01,
	0x0831, 0x01,
	0x0832, 0x10,
	0x0833, 0x06,
	0x0834, 0x02,
	0x0835, 0x00,
	0x0836, 0x00,
	0x0837, 0x01,
	0x0838, 0x01,
	0x0839, 0x01,
	0x083a, 0x0e,
	0x083b, 0x06,
	0x083c, 0x02,
	0x083d, 0x00,
	0x083e, 0x00,
	0x083f, 0x01,
	0x0840, 0x01,
	0x0841, 0x01,
	0x0842, 0x0a,
	0x0843, 0x06,
	0x0844, 0x02,
	0x0845, 0x00,
	0x0846, 0x00,
	0x0847, 0x01,
	0x0848, 0x01,
	0x0849, 0x01,
	0x084a, 0x02,
	0x084b, 0x06,
	0x084c, 0x02,
	0x084d, 0x00,
	0x084e, 0x00,
	0x084f, 0x01,
	0x0850, 0x01,
	0x0851, 0x00,
	0x0852, 0x1c,
	0x0853, 0x06,
	0x0854, 0x02,
	0x0855, 0x00,
	0x0856, 0x00,
	0x0857, 0x01,
	0x0858, 0x01,
	0x0859, 0x00,
	0x085a, 0x16,
	0x085b, 0x06,
	0x085c, 0x02,
	0x085d, 0x02,
	0x085e, 0x00,
	0x085f, 0x01,
	0x0860, 0x01,
	0x0861, 0x00,
	0x0862, 0x0a,
	0x0863, 0x06,
	0x0864, 0x02,
	0x0865, 0x02,
	0x0866, 0x00,
	0x0867, 0x01,
	0x0868, 0x01,
	0x0869, 0x00,
	0x086a, 0x00,
	0x086b, 0x01,
	0x086c, 0x00,
	0x086d, 0x01,
	0x086e, 0x00,
	0x086f, 0x00,
	0x0870, 0x01,
	0x0871, 0x01,
	0x0872, 0x61,
	0x0873, 0x00,
	0x0874, 0x02,
	0x0875, 0x01,
	0x0876, 0xf4,
	0x0877, 0x00,
	0x0878, 0x03,
	0x0879, 0x02,
	0x087a, 0xbb,
	0x087b, 0x00,
	0x087c, 0x04,
	0x087d, 0x03,
	0x087e, 0xe5,
	0x087f, 0x00,
	0x0880, 0x05,
	0x0881, 0x05,
	0x0882, 0x73,
	0x0883, 0x00,
	0x0884, 0x06,
	0x0885, 0x07,
	0x0886, 0xd6,
	0x0887, 0x10,
	0x0888, 0x05,
	0x0889, 0x0a,
	0x088a, 0xfb,
	0x088b, 0x10,
	0x088c, 0x06,
	0x088d, 0x0f,
	0x088e, 0x89,
	0x088f, 0x14,
	0x0890, 0xb6,
	0x0891, 0x1f,
	0x0892, 0x90,
	0x0893, 0x1a,
	0x0894, 0x66,
	0x0895, 0x01,
	0x0896, 0x46,
	0x0897, 0x02,
	0x0898, 0x01,
	0x0899, 0x01,
	0x089a, 0x01,
	0x089b, 0x03,
	0x089c, 0x4c,
	0x089d, 0x04,
	0x089e, 0xff,
	0x089f, 0xff,
	0x08a0, 0x99,
	0x08a1, 0x02,
	0x08a2, 0x02,
	0x08a3, 0x04,
	0x08a4, 0x02,
	0x08a5, 0x0e,
	0x08a6, 0x02,
	0x08a7, 0x03,
	0x08a8, 0x40,
	0x08a9, 0x04,
	0x08aa, 0xff,
	0x08ab, 0xff,
	0x08ac, 0x00,
	0x05ac, 0x01,
	0x0207, 0xc4,
	0x05a0, 0xc2,
	0x01c0, 0x01,
	0x0096, 0x81,
	0x0097, 0x08,
	0x0098, 0x87,
	0x0204, 0x04,
	0x0205, 0x00,
	0x0208, 0x01,
	0x0209, 0x6f,
	0x0351, 0x00,
	0x0352, 0xe4,
	0x0353, 0x00,
	0x0354, 0x0c,
	0x034c, 0x09,
	0x034d, 0x00,
	0x034e, 0x05,
	0x034f, 0x10,
	0x0180, 0x46,
	0x0181, 0xf0,
	0x0185, 0x01,
	0x0103, 0x10,
	0x0106, 0x39,
	0x0114, 0x03,
	0x0115, 0x20,
	0x0121, 0x05,
	0x0122, 0x07,
	0x0123, 0x17,
	0x0124, 0x00,
	0x0125, 0x16,
	0x0126, 0x08,
	0x0128, 0xf0,
	0x0129, 0x07,
	0x012a, 0x09,
	0x012b, 0x09,
	0x0a70, 0x11,
	0x0313, 0x80,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x03fe, 0x00,
	0x0a70, 0x00,
	0x0070, 0x05,
	0x0089, 0x03,
	0x009b, 0x40,
	0x00a4, 0x80,
	0x00a0, 0x05,
	0x00a6, 0x07,
	0x0080, 0xd2,
	0x00c1, 0x80,
	0x00c2, 0x11,
	0x024d, 0x01,
	0x0084, 0x10,
	0x0268, 0x00,
	0x031c, 0x9f,
};

#endif
