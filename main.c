/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief  PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 *
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"
#include "nrf_gpio.h"
#include "nrfx_twi.h"
#include "nrf_twi.h"

// Define TWI instance ID before using it
#define TWI_INSTANCE_ID 0

static const nrfx_twi_t i2c = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

// Define GPIO pins and TWI pins
#define GPIO_PIN_27 27 //NE JAMAIS ALLUMER
#define GPIO_PIN_26 26 //NE JAMAIS ALLUMER

#define SCL_PIN NRF_GPIO_PIN_MAP(0,18)
#define SDA_PIN NRF_GPIO_PIN_MAP(0,17)

#define GPIO_PIN 24

/** CODE POUR LE BRUMISATEUR **/
#define BRUMISATEUR_PIN NRF_GPIO_PIN_MAP(0,29)


void brumisateur_init(){
	nrf_gpio_cfg_output(BRUMISATEUR_PIN);
}

void brumisateur_on(){

	nrf_gpio_pin_set(BRUMISATEUR_PIN);
}

void brumisateur_off(){

	nrf_gpio_pin_clear(BRUMISATEUR_PIN);

}
/** FIN DU CODE DU BRUMISATEUR **/


/**  CODE POUR LE VEML6030 */

// VEML6030 I2C address when ADDR pin is connected to ground
#define VEML6030_I2C_ADDRESS       0x10
#define VEML6030_RESOLUTION 0.0336

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
uint16_t veml6030_read_data(void);
uint16_t veml6030_read_id(void);
void veml6030_configure(uint16_t config);

void veml6030_init(void) {
	// Configuration initiale du VEML6030
	uint16_t config = VEML6030_ALS_GAIN_2 | VEML6030_ALS_IT_100MS | VEML6030_ALS_PERS_1 | VEML6030_ALS_POWER_ON;
	veml6030_configure(config);
}

uint16_t veml6030_read_id(void) {
	uint8_t id_data[2];
	uint8_t reg = VEML6030_REG_ID;

	nrfx_twi_tx(&i2c, VEML6030_I2C_ADDRESS, &reg, 1, true);
	nrfx_twi_rx(&i2c, VEML6030_I2C_ADDRESS, id_data, 2);
	return (id_data[1] << 8) | id_data[0];
}

void veml6030_configure(uint16_t config) {
	uint8_t config_data[3] = {VEML6030_REG_ALS_CONF, config & 0xFF, config >> 8};
	nrfx_twi_tx(&i2c, VEML6030_I2C_ADDRESS, config_data, 3, false);
}

uint16_t veml6030_read_data(void){
	uint8_t received_data[2];
	uint8_t data_reg = VEML6030_REG_ALS;
	//Send the register address to read from
	nrfx_twi_tx(&i2c, VEML6030_I2C_ADDRESS, &data_reg, 1, true);

	//Read the ALS data
	nrfx_twi_rx(&i2c, VEML6030_I2C_ADDRESS, received_data, 2);

	// Combine the two bytes into a 16-bit value
	return (received_data[1] << 8) | received_data[0];
}

uint16_t veml6030_get_lx(uint16_t data){

	return data*VEML6030_RESOLUTION;

}
/** FIN DU CODE POUR LE VEML6030  */


/** DEBUT DU CODE POUR LE ZMOD4410 **/

#define ZMOD4410_SLAVE_ADDRESS 0x32

#define ZMOD4410_REG_ID 0x00
#define ZMOD4410_REG_CONF 0x20
#define ZMOD4410_REG_CMD 0x93
#define ZMOD4410_REG_STATUS 0x94
#define ZMOD4410_REG_TEST 0x88

uint16_t zmod4410_write_then_read(){

	uint8_t received_data[2];
	uint8_t data_reg[2] = { ZMOD4410_REG_TEST, 0x52 };
	uint8_t reg = ZMOD4410_REG_TEST;

	nrfx_twi_tx(&i2c, ZMOD4410_SLAVE_ADDRESS, data_reg,2, false); //Envoyer 0x02 dans le registre 0x88

	nrf_delay_ms(5);

	nrfx_twi_tx(&i2c, ZMOD4410_SLAVE_ADDRESS, &reg, 1, true );
	nrfx_twi_rx(&i2c, ZMOD4410_SLAVE_ADDRESS, received_data, 2); //lire le registre

	return (received_data[1] << 8) | received_data[0];
}

uint16_t zmod4410_read_id() {
	uint8_t id_data[2];
	uint8_t reg_addr = ZMOD4410_REG_ID;
	ret_code_t err_code;

	// Send the register address to read from
	err_code = nrfx_twi_tx(&i2c, ZMOD4410_SLAVE_ADDRESS, &reg_addr, 1, true);
	APP_ERROR_CHECK(err_code);

	// Read the ID register
	err_code = nrfx_twi_rx(&i2c, ZMOD4410_SLAVE_ADDRESS, id_data, 2);
	APP_ERROR_CHECK(err_code);

	return (id_data[1] << 8) | id_data[0];
}




/** FIN DU CODE POUR LE ZMOD4410 **/

/** DEBUT DU CODE POUR LE HS3003 **/

#define HS3003_SLAVE_ADDR 0x44
#define humidity_formula(x) ((x*100)/16383)
#define temperature_formula(x)  ((x*165)/16383) - 40

typedef struct {
	uint16_t temperature;
	uint16_t humidity;
} hs300x_data_t;

void hs3003_init(){

	nrfx_twi_tx(&i2c, HS3003_SLAVE_ADDR, 0x0, 1, false); //Sort du sleep mode
}

uint16_t hs3003_read_id(){

	uint8_t id_data[2];
	uint8_t prog_mode_enter[3] = {0xA0, 0x00, 0x00}; //Programming mode in
	uint8_t prog_mode_out[3] = {0x80, 0x00, 0x00}; //Programming mode out
	ret_code_t err_code;

	//Entre dans le mode programming
	err_code = nrfx_twi_tx(&i2c, HS3003_SLAVE_ADDR, prog_mode_enter, 3, false);
	APP_ERROR_CHECK(err_code);



	// Read the ID register
	err_code = nrfx_twi_rx(&i2c, HS3003_SLAVE_ADDR, id_data	, 1); //LSB
	APP_ERROR_CHECK(err_code);

	//Sort du mode programming
	err_code = nrfx_twi_tx(&i2c, HS3003_SLAVE_ADDR, prog_mode_out, 3, false);
	APP_ERROR_CHECK(err_code);


	return (id_data[1] << 8) | id_data[0];
}

hs300x_data_t hs3003_get_measure(){

	ret_code_t err_code;
	uint8_t measure[4];
	hs300x_data_t resultat;
	//uint8_t reg = 0x00;

	//err_code = nrfx_twi_tx(&i2c, HS3003_SLAVE_ADDR, &reg, 1, false);
	//APP_ERROR_CHECK(err_code);

	// Délai pour assurer que l'écriture est terminée avant la lecture
	//nrf_delay_ms(10);

	err_code = nrfx_twi_rx(&i2c, HS3003_SLAVE_ADDR, measure, 4); //Lecture
	APP_ERROR_CHECK(err_code);

	nrf_delay_ms(40);

	resultat.humidity = ((measure[0] << 8) | measure[1]) & 0x3FFF; //Ok
	resultat.temperature = (((measure[2] << 8) | measure[3]) & 0xFFFC) >> 2 ; //à l'air ok

	return resultat;

}

float hs3003_get_humidity(hs300x_data_t results){

	return humidity_formula(results.humidity);
}

float hs3003_get_temperature(hs300x_data_t results){

	return temperature_formula(results.temperature);
}



/** FIN DU CODE POUR LE HS3003 **/

/** CODE POUR LA SONDE pH **/




/** FIN DU CODE POUR LA SONDE pH **/



void twi_init(void) {
	uint8_t err_code;

	const nrfx_twi_config_t i2c_cfg = {
			.scl                = SCL_PIN,
			.sda                = SDA_PIN,
			.frequency          = NRF_TWI_FREQ_400K,
			.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
			.hold_bus_uninit    = false
	};

	err_code = nrfx_twi_init(&i2c, &i2c_cfg, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrfx_twi_enable(&i2c);
}



int main(void)
{
	// Configure la broche comme sortie
	//nrf_gpio_cfg_output(GPIO_PIN);

	// Initialize I2C
	twi_init();

	// Initialize VEML6030
	veml6030_init();

	//Initialise ZMOD4410

	//Initialise HS3003
	hs3003_init();

	brumisateur_init();

	hs300x_data_t humidity_temperature;

	uint16_t lux;
	//uint16_t value;
	float humidite;
	float temperature;


	while (true)
	{
		hs3003_init();
		humidity_temperature = hs3003_get_measure();

		lux = veml6030_get_lx(veml6030_read_data());
		//value = zmod4410_write_then_read();
		humidite = hs3003_get_humidity(humidity_temperature);
		temperature = hs3003_get_temperature(humidity_temperature);


		if(temperature > 25 && humidite < 50){
			nrf_gpio_pin_set(GPIO_PIN);
		}
		else if(humidite < 60){
			brumisateur_on();
		}
		else if(lux < 2000){
			//Notification "Pas assez de lumière"
		}

		else{

			//brumisateur_off();
			nrf_gpio_pin_clear(GPIO_PIN);

		} // end if
		nrf_delay_ms(1000);  // Delay
	} // end while
}

/** @} */
