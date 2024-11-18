/*
 * veml6030.h
 *
 *  Created on: 6 juin 2024
 *      Author: Kévin Pottier & Johann Raineteau
 */

#ifndef VEML6030_H
#define VEML6030_H

#include <stdint.h>


// VEML6030 I2C address when ADDR pin is connected to ground
#define VEML6030_I2C_ADDRESS       0x10

// Register addresses
#define VEML6030_REG_ALS_CONF      0x00
#define VEML6030_REG_ALS_WH        0x01
#define VEML6030_REG_ALS_WL        0x02
#define VEML6030_REG_POWER_SAVING  0x03
#define VEML6030_REG_ALS           0x04
#define VEML6030_REG_WHITE         0x05
#define VEML6030_REG_ALS_INT       0x06
#define VEML6030_REG_ID            0x07

// Configuration register bits
#define VEML6030_ALS_GAIN_MASK     (0x03 << 11)
#define VEML6030_ALS_IT_MASK       (0x0F << 6)
#define VEML6030_ALS_PERS_MASK     (0x03 << 4)
#define VEML6030_ALS_INT_EN_MASK   (0x01 << 1)
#define VEML6030_ALS_SD_MASK       (0x01 << 0)

// Gain settings
#define VEML6030_ALS_GAIN_1        (0x00 << 11)
#define VEML6030_ALS_GAIN_2        (0x01 << 11)
#define VEML6030_ALS_GAIN_1_8      (0x02 << 11)
#define VEML6030_ALS_GAIN_1_4      (0x03 << 11)

// Integration time settings
#define VEML6030_ALS_IT_25MS       (0x0C << 6)
#define VEML6030_ALS_IT_50MS       (0x08 << 6)
#define VEML6030_ALS_IT_100MS      (0x00 << 6)
#define VEML6030_ALS_IT_200MS      (0x01 << 6)
#define VEML6030_ALS_IT_400MS      (0x02 << 6)
#define VEML6030_ALS_IT_800MS      (0x03 << 6)

// ALS persistence protect number setting
#define VEML6030_ALS_PERS_1        (0x00 << 4)
#define VEML6030_ALS_PERS_2        (0x01 << 4)
#define VEML6030_ALS_PERS_4        (0x02 << 4)
#define VEML6030_ALS_PERS_8        (0x03 << 4)

// Interrupt enable setting
#define VEML6030_ALS_INT_DISABLE   (0x00 << 1)
#define VEML6030_ALS_INT_ENABLE    (0x01 << 1)

// ALS shutdown setting
#define VEML6030_ALS_POWER_ON      (0x00 << 0)
#define VEML6030_ALS_SHUTDOWN      (0x01 << 0)

// Function prototypes
void veml6030_init(void);
uint16_t veml6030_read_als(void);
uint16_t veml6030_read_id(void);
void veml6030_configure(uint16_t config);

#endif // VEML6030_H

