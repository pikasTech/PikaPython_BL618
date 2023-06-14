#ifndef _TOUCH_CONF_USER_H_
#define _TOUCH_CONF_USER_H_

/* i2c interface
    TOUCH_I2C_FT6X36
    TOUCH_I2C_GT911
*/
/* Select Touch Type */
#define TOUCH_I2C_GT911

/* touch interface */
#define TOUCH_INTERFACE_SPI 1
#define TOUCH_INTERFACE_I2C 2

/* touch interface pin config */
#define TOUCH_I2C_SCL_PIN   GPIO_PIN_14
#define TOUCH_I2C_SDA_PIN   GPIO_PIN_15

#endif // _TOUCH_CONF_USER_H_