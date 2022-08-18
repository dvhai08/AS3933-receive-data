#ifndef AS3933_H
#define AS3933_H

#include "stdint.h"
#include "stdbool.h"

// MODE | REG ADDR | REG_DATA
// 2    | 6	      | 8

/* R0 */
#define AS3933_R0		0x00

#define AS3933_EN_A1		0x02
#define AS3933_EN_A3		0x04
#define AS3933_EN_A2		0x08
#define AS3933_MUX123		0x10
#define AS3933_ON_OFF		0x20
#define AS3933_DAT_MASK	0x40
#define AS3933_PAT32		0x80


/* R1 */
#define AS3933_R1		0x01

#define AS3933_EN_XTAL	0x01
#define AS3933_EN_WPAT	0x02
#define AS3933_EN_PAT2	0x04
#define AS3933_EN_MANCH	0x08
#define AS3933_ATT_ON		0x10
#define AS3933_AGC_UD		0x20
#define AS3933_AGC_TLIM	0x40
#define AS3933_ABS_HY		0x80

/* R2 */
#define AS3933_R2 	0x02

#define AS3933_S_WU1_MASK	0x03
#define AS3933_SET_S_WU1(x)	(AS3933_S_WU1_MASK & (x))
#define AS3933_G_BOOST		0x20
#define AS3933_EN_EXT_CLK	0x40
#define AS3933_S_ABS		0x80


/* R3 */
#define AS3933_R3 	0x03

#define AS3933_FS_ENV_MASK		0x07
#define AS3933_SET_FS_ENV(x)	(AS3933_FS_ENV_MASK & (x)) // R3<2:0>
#define AS3933_FS_SCL_MASK		0x38
#define AS3933_SET_FS_SCL(x)	(AS3933_FS_SCL_MASK & (x << 3)) // R3<5:3>

#define AS3933_HY_POS			0x40
#define AS3933_HY_20m			0x80

/* R4 */
#define AS3933_R4 	0x04

#define AS3933_GR_MASK			0x0F
#define AS3933_SET_GR(x)	(AS3933_GR_MASK & (x))
#define AS3933_D_RES_MASK		0x30
#define AS3933_SET_D_RES(x)		(AS3933_D_RES_MASK & (x << 4))
#define AS3933_T_OFF_MASK		0xC0
#define AS3933_SET_T_OFF(x)		(AS3933_T_OFF_MASK & (x << 6))

/* R5 */
#define AS3933_R5 	0x05
/* R6 */
#define AS3933_R6 	0x06

/* R7 */
#define AS3933_R7 	0x07

#define AS3933_T_HBIT_MASK			0x1F
#define AS3933_SET_T_HBIT(x)	(AS3933_T_HBIT_MASK & (x))
#define AS3933_T_OUT_MASK				0xE0
#define AS3933_SET_T_OUT(x)	(AS3933_T_OUT_MASK & (x << 5))

/* R8 */
#define AS3933_R8 	0x08

#define AS3933_T_AUTO_MASK		0x07
#define AS3933_SET_T_AUTO(x)	(AS3933_T_AUTO_MASK & (x))
#define AS3933_BAND_SEL_MASK		0xE0
#define AS3933_SET_BAND_SEL(x)	(AS3933_BAND_SEL_MASK & (x << 5))

/* R9 */
#define AS3933_R9 	0x09
#define AS3933_BLOCK_AGC	0x80

/* R10 */
#define AS3933_R10	0x0A

#define AS3933_RSSI1_MASK	0x1F
#define AS3933_GET_RSSI1(x)	(AS3933_RSSI1_MASK & (x))

/* R11 */
#define AS3933_R11 	0x0B

#define AS3933_RSSI3_MASK	0x1F
#define AS3933_GET_RSSI3(x)	(AS3933_RSSI3_MASK & (x))

/* R12 */
#define AS3933_R12 	0x0C

#define AS3933_RSSI2_MASK	0x1F
#define AS3933_GET_RSSI2(x)	(AS3933_RSSI2_MASK & (x))

/* R13 */
#define AS3933_R13 	0x0D

/* R14 */
#define AS3933_R14 	0x0E

// x: 0 - 64
#define AS3933_RC_OSC_TAPS_MASK		0x3F
#define AS3933_SET_RC_OSC_TAPS(x)	(AS3933_RC_OSC_TAPS_MASK & (x))

#define AS3933_RC_CAL_OK			0x40
#define AS3933_RC_CAL_KO			0x80

/* R15 */
#define AS3933_R15 	0x0F

#define AS3933_LC_OSC_OK			0x08
#define AS3933_LC_OSC_KO			0x10

/* R16 */
#define AS3933_R16 	0x10

#define AS3933_LC_OSC_MUX1	0x01
#define AS3933_LC_OSC_MUX2	0x02
#define AS3933_LC_OSC_MUX3	0x04
#define AS3933_RC_OSC_MAX		0x10
#define AS3933_RC_OSC_MIN		0x20
#define AS3933_LC_OSC_DIS		0x40
#define AS3933_CLOCK_GEN_DIS	0x80

/* R17 */
#define AS3933_R17 		0x11

#define AS3933_CAP_CH1_MASK		0x1F
#define AS3933_SET_CAP_CH1(x)	(AS3933_CAP_CH1_MASK & (x))
/* R18 */
#define AS3933_R18 	0x12

#define AS3933_CAP_CH2_MASK		0x1F
#define AS3933_SET_CAP_CH2(x)	(AS3933_CAP_CH2_MASK & (x))
/* R19 */
#define AS3933_R19 	0x13

#define AS3933_CAP_CH3_MASK		0x1F
#define AS3933_CAP_CH3_1pF		0x01
#define AS3933_CAP_CH3_2pF		0x02
#define AS3933_CAP_CH3_4pF		0x04
#define AS3933_CAP_CH3_8pF		0x08
#define AS3933_CAP_CH3_16pF		0x10
#define AS3933_SET_CAP_CH3(x)	(AS3933_CAP_CH3_MASK & (x))

/* DIRECT COMMANDS */
#define AS3933_CLEAR_WAKE		0x00
#define AS3933_RESET_RSSI		0x01
#define AS3933_CALIB_RCOSC	0x02
#define AS3933_CLEAR_FALSE	0x03
#define AS3933_PRESET_DEFAULT	0x04
#define AS3933_CALIB_RCO_LC	0x05

void as3933_write_Rx(uint8_t reg, uint8_t val);
uint8_t as3933_read_Rx(uint8_t reg);
void as3933_write_direct(uint8_t cmd);

void as3933_set_pattern_16(uint8_t *pattern);
void as3933_get_pattern_16(uint8_t *pattern);
void as3933_set_listening_mode(void);
void as3933_reset_rssi(void);

#endif
