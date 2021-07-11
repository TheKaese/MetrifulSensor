/* 
   Metriful_sensor.h

   Copyright 2020 Metriful Ltd. 
   Licensed under the MIT License - for further details see LICENSE.txt

   For code examples, datasheet and user guide, visit 
   https://github.com/metriful/sensor
*/
#include "Metriful_sensor.h"

void SensorHardwareSetup(uint8_t i2c_7bit_address)
{
  // READY, light interrupt and sound interrupt lines are digital inputs.
  gpio_config_t ready_pin_conf;
  ready_pin_conf.intr_type = GPIO_INTR_ANYEDGE;
  ready_pin_conf.mode = GPIO_MODE_INPUT;
  ready_pin_conf.pin_bit_mask = (1ULL << READY_PIN); //23
  ready_pin_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ready_pin_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&ready_pin_conf));

  gpio_config_t l_int_pint_conf;
  l_int_pint_conf.intr_type = GPIO_INTR_DISABLE;
  l_int_pint_conf.mode = GPIO_MODE_INPUT;
  l_int_pint_conf.pin_bit_mask = (1ULL << L_INT_PIN); //18
  l_int_pint_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  l_int_pint_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&l_int_pint_conf));

  gpio_config_t s_int_pint_conf;
  s_int_pint_conf.intr_type = GPIO_INTR_DISABLE;
  s_int_pint_conf.mode = GPIO_MODE_INPUT;
  s_int_pint_conf.pin_bit_mask = (1ULL << S_INT_PIN); //19
  s_int_pint_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  s_int_pint_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&s_int_pint_conf));

  // Set up interrupt monitoring of the READY signal, triggering on a falling edge
  // event (high-to-low voltage change) indicating READY assertion. The
  // function ready_ISR() will be called when this happens.
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_EDGE));
  gpio_isr_handler_add(READY_PIN, &ready_isr, (void *)READY_PIN);

  // Init I2C
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = SDA_PIN; //21
  conf.scl_io_num = SCL_PIN; //22
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.clk_flags = I2C_SCLK_DEFAULT;
  conf.master.clk_speed = 100000;
  ESP_LOGI(TAG, "I2C Param Config");
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_LOGI(TAG, "I2C Driver Install");
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

  ESP_LOGI(TAG, "Waiting for Metriful to finish initialization...");
  // Wait for the MS430 to finish power-on initialization:
  while (gpio_get_level(READY_PIN) > .5)
  {
    vTaskDelay(5 / portTICK_RATE_MS);
  }

  // Reset to clear any previous state:
  uint8_t data = 0;
  TransmitI2C(i2c_7bit_address, RESET_CMD, &data, 1);

  ESP_LOGI(TAG, "Waiting for Metriful reset");
  // Wait for reset completion and entry to standby mode
  while (gpio_get_level(GPIO_NUM_23) > .5)
  {
    vTaskDelay(5 / portTICK_RATE_MS);
  }
  ESP_LOGI(TAG, "Metriful hardware setup complete");
}

volatile bool ready_assertion_event = false;

// This function is automatically called after a falling edge (assertion) of READY.
// The flag variable is set true - it must be set false again in the main program.
void ISR_ATTRIBUTE ready_isr(void *arg)
{
  ESP_LOGI(TAG, "Metriful");
  ready_assertion_event = true;
}

////////////////////////////////////////////////////////////////////////

// Functions to convert data from integer representation to floating-point representation.
// Floats are easy to use for writing programs but require greater memory and processing
// power resources, so may not always be appropriate.

void convertAirDataF(const AirData_t *airData_in, AirData_F_t *airDataF_out)
{
  // Decode the signed value for T (in Celsius)
  airDataF_out->T_C = convertEncodedTemperatureToFloat(airData_in->T_C_int_with_sign,
                                                       airData_in->T_C_fr_1dp);
  airDataF_out->P_Pa = airData_in->P_Pa;
  airDataF_out->H_pc = ((float)airData_in->H_pc_int) + (((float)airData_in->H_pc_fr_1dp) / 10.0);
  airDataF_out->G_Ohm = airData_in->G_ohm;
}

void convertAirQualityDataF(const AirQualityData_t *airQualityData_in, AirQualityData_F_t *airQualityDataF_out)
{
  airQualityDataF_out->AQI = ((float)airQualityData_in->AQI_int) +
                             (((float)airQualityData_in->AQI_fr_1dp) / 10.0);
  airQualityDataF_out->CO2e = ((float)airQualityData_in->CO2e_int) +
                              (((float)airQualityData_in->CO2e_fr_1dp) / 10.0);
  airQualityDataF_out->bVOC = ((float)airQualityData_in->bVOC_int) +
                              (((float)airQualityData_in->bVOC_fr_2dp) / 100.0);
  airQualityDataF_out->AQI_accuracy = airQualityData_in->AQI_accuracy;
}

void convertLightDataF(const LightData_t *lightData_in, LightData_F_t *lightDataF_out)
{
  lightDataF_out->illum_lux = ((float)lightData_in->illum_lux_int) +
                              (((float)lightData_in->illum_lux_fr_2dp) / 100.0);
  lightDataF_out->white = lightData_in->white;
}

void convertSoundDataF(const SoundData_t *soundData_in, SoundData_F_t *soundDataF_out)
{
  soundDataF_out->SPL_dBA = ((float)soundData_in->SPL_dBA_int) +
                            (((float)soundData_in->SPL_dBA_fr_1dp) / 10.0);
  for (uint16_t i = 0; i < SOUND_FREQ_BANDS; i++)
  {
    soundDataF_out->SPL_bands_dB[i] = ((float)soundData_in->SPL_bands_dB_int[i]) +
                                      (((float)soundData_in->SPL_bands_dB_fr_1dp[i]) / 10.0);
  }
  soundDataF_out->peakAmp_mPa = ((float)soundData_in->peak_amp_mPa_int) +
                                (((float)soundData_in->peak_amp_mPa_fr_2dp) / 100.0);
  soundDataF_out->stable = (soundData_in->stable == 1);
}

void convertParticleDataF(const ParticleData_t *particleData_in, ParticleData_F_t *particleDataF_out)
{
  particleDataF_out->duty_cycle_pc = ((float)particleData_in->duty_cycle_pc_int) +
                                     (((float)particleData_in->duty_cycle_pc_fr_2dp) / 100.0);
  particleDataF_out->concentration = ((float)particleData_in->concentration_int) +
                                     (((float)particleData_in->concentration_fr_2dp) / 100.0);
  particleDataF_out->valid = (particleData_in->valid == 1);
}

// Send data to the Metriful MS430 using the I2C-compatible two wire interface.
//
// Returns true on success, false on failure.
//
// dev_addr_7bit = the 7-bit I2C address of the MS430 board.
// commandRegister = the settings register code or command code to be used.
// data = array containing the data to be sent; its length must be at least "data_length" bytes.
// data_length = the number of bytes from the "data" array to be sent.
//
bool TransmitI2C(uint8_t dev_addr_7bit, uint8_t commandRegister, uint8_t data[], uint8_t data_length)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  i2c_master_write_byte(cmd, (0x71 << 1) | I2C_MASTER_WRITE, 1);

  if (data_length > 0)
  {
    i2c_master_write(cmd, data, data_length, 1);
  }

  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd);

  return 1;
}

// Read data from the Metriful MS430 using the I2C-compatible two wire interface.
//
// Returns true on success, false on failure.
//
// dev_addr_7bit = the 7-bit I2C address of the MS430 board.
// commandRegister = the settings register code or data location code to be used.
// data = array to store the received data; its length must be at least "data_length" bytes.
// data_length = the number of bytes to read.
//
bool ReceiveI2C(uint8_t dev_addr_7bit, uint8_t commandRegister, uint8_t data[], uint8_t data_length)
{
  // Cannot do a zero byte read
  if (data_length == 0)
  {
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr_7bit << 1) | I2C_MASTER_WRITE, 1 /* expect ack */));
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr_7bit << 1) | I2C_MASTER_READ, 1 /* expect ack */));

  ESP_ERROR_CHECK(i2c_master_read(cmd, data, data_length, I2C_MASTER_ACK));

  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd);

  return true;
}

////////////////////////////////////////////////////////////////////////

// Provide a readable interpretation of the accuracy code for
// the air quality measurements (applies to all air quality data)
const char *interpret_AQI_accuracy(uint8_t AQI_accuracy_code)
{
  switch (AQI_accuracy_code)
  {
  default:
  case 0:
    return "Not yet valid, self-calibration incomplete";
  case 1:
    return "Low accuracy, self-calibration ongoing";
  case 2:
    return "Medium accuracy, self-calibration ongoing";
  case 3:
    return "High accuracy";
  }
}

// Provide a readable interpretation of the AQI (air quality index)
const char *interpret_AQI_value(uint16_t AQI)
{
  if (AQI < 50)
  {
    return "Good";
  }
  else if (AQI < 100)
  {
    return "Acceptable";
  }
  else if (AQI < 150)
  {
    return "Substandard";
  }
  else if (AQI < 200)
  {
    return "Poor";
  }
  else if (AQI < 300)
  {
    return "Bad";
  }
  else
  {
    return "Very bad";
  }
}

// Set the threshold for triggering a sound interrupt.
//
// Returns true on success, false on failure.
//
// threshold_mPa = peak sound amplitude threshold in milliPascals, any 16-bit integer is allowed.
bool setSoundInterruptThreshold(uint8_t dev_addr_7bit, uint16_t threshold_mPa)
{
  uint8_t TXdata[SOUND_INTERRUPT_THRESHOLD_BYTES] = {0};
  TXdata[0] = (uint8_t)(threshold_mPa & 0x00FF);
  TXdata[1] = (uint8_t)(threshold_mPa >> 8);
  return TransmitI2C(dev_addr_7bit, SOUND_INTERRUPT_THRESHOLD_REG, TXdata, SOUND_INTERRUPT_THRESHOLD_BYTES);
}

// Set the threshold for triggering a light interrupt.
//
// Returns true on success, false on failure.
//
// The threshold value in lux units can be fractional and is formed as:
//     threshold = thres_lux_int + (thres_lux_fr_2dp/100)
//
// Threshold values exceeding MAX_LUX_VALUE will be limited to MAX_LUX_VALUE.
bool setLightInterruptThreshold(uint8_t dev_addr_7bit, uint16_t thres_lux_int, uint8_t thres_lux_fr_2dp)
{
  uint8_t TXdata[LIGHT_INTERRUPT_THRESHOLD_BYTES] = {0};
  TXdata[0] = (uint8_t)(thres_lux_int & 0x00FF);
  TXdata[1] = (uint8_t)(thres_lux_int >> 8);
  TXdata[2] = thres_lux_fr_2dp;
  return TransmitI2C(dev_addr_7bit, LIGHT_INTERRUPT_THRESHOLD_REG, TXdata, LIGHT_INTERRUPT_THRESHOLD_BYTES);
}

////////////////////////////////////////////////////////////////////////

// Convenience functions for reading data (integer representation)
//
// For each category of data (air, sound, etc.) a pointer to the data
// struct is passed to the ReceiveI2C() function. The received byte
// sequence fills the data struct in the correct order so that each
// field within the struct receives the value of an environmental data
// quantity (temperature, sound level, etc.)
SoundData_t getSoundData(uint8_t i2c_7bit_address)
{
  SoundData_t soundData;
  ReceiveI2C(i2c_7bit_address, SOUND_DATA_READ, (uint8_t *)&soundData, SOUND_DATA_BYTES);
  return soundData;
}

AirData_t getAirData(uint8_t i2c_7bit_address)
{
  AirData_t airData;
  ReceiveI2C(i2c_7bit_address, AIR_DATA_READ, (uint8_t *)&airData, AIR_DATA_BYTES);
  return airData;
}

LightData_t getLightData(uint8_t i2c_7bit_address)
{
  LightData_t lightData;
  ReceiveI2C(i2c_7bit_address, LIGHT_DATA_READ, (uint8_t *)&lightData, LIGHT_DATA_BYTES);
  return lightData;
}

AirQualityData_t getAirQualityData(uint8_t i2c_7bit_address)
{
  AirQualityData_t airQualityData;
  ReceiveI2C(i2c_7bit_address, AIR_QUALITY_DATA_READ, (uint8_t *)&airQualityData, AIR_QUALITY_DATA_BYTES);
  return airQualityData;
}

ParticleData_t getParticleData(uint8_t i2c_7bit_address)
{
  ParticleData_t particleData;
  ReceiveI2C(i2c_7bit_address, PARTICLE_DATA_READ, (uint8_t *)&particleData, PARTICLE_DATA_BYTES);
  return particleData;
}

// Convenience functions for reading data (float representation)

SoundData_F_t getSoundDataF(uint8_t i2c_7bit_address)
{
  SoundData_F_t soundDataF;
  SoundData_t soundData = getSoundData(i2c_7bit_address);
  convertSoundDataF(&soundData, &soundDataF);
  return soundDataF;
}

AirData_F_t getAirDataF(uint8_t i2c_7bit_address)
{
  AirData_F_t airDataF;
  AirData_t airData = getAirData(i2c_7bit_address);
  convertAirDataF(&airData, &airDataF);
  return airDataF;
}

LightData_F_t getLightDataF(uint8_t i2c_7bit_address)
{
  LightData_F_t lightDataF;
  LightData_t lightData = getLightData(i2c_7bit_address);
  convertLightDataF(&lightData, &lightDataF);
  return lightDataF;
}

AirQualityData_F_t getAirQualityDataF(uint8_t i2c_7bit_address)
{
  AirQualityData_F_t airQualityDataF;
  AirQualityData_t airQualityData = getAirQualityData(i2c_7bit_address);
  convertAirQualityDataF(&airQualityData, &airQualityDataF);
  return airQualityDataF;
}

ParticleData_F_t getParticleDataF(uint8_t i2c_7bit_address)
{
  ParticleData_F_t particleDataF;
  ParticleData_t particleData = getParticleData(i2c_7bit_address);
  convertParticleDataF(&particleData, &particleDataF);
  return particleDataF;
}

////////////////////////////////////////////////////////////////////////

// Functions to convert Celsius temperature to Fahrenheit, in float
// and integer formats

float convertCtoF(float C)
{
  return ((C * 1.8) + 32.0);
}

// Convert Celsius to Fahrenheit in sign, integer and fractional parts
void convertCtoF_int(float C, uint8_t *F_int, uint8_t *F_fr_1dp, bool *isPositive)
{
  float F = convertCtoF(C);
  bool isNegative = (F < 0.0);
  if (isNegative)
  {
    F = -F;
  }
  F += 0.05;
  F_int[0] = (uint8_t)F;
  F -= (float)F_int[0];
  F_fr_1dp[0] = (uint8_t)(F * 10.0);
  isPositive[0] = (!isNegative);
}

// Decode and convert the temperature as read from the MS430 (integer
// representation) into a float value
float convertEncodedTemperatureToFloat(uint8_t T_C_int_with_sign, uint8_t T_C_fr_1dp)
{
  float temperature_C = ((float)(T_C_int_with_sign & TEMPERATURE_VALUE_MASK)) +
                        (((float)T_C_fr_1dp) / 10.0);
  if ((T_C_int_with_sign & TEMPERATURE_SIGN_MASK) != 0)
  {
    // the most-significant bit is set, indicating that the temperature is negative
    temperature_C = -temperature_C;
  }
  return temperature_C;
}

// Obtain temperature, in chosen units (C or F), as sign, integer and fractional parts
const char *getTemperature(const AirData_t *pAirData, uint8_t *T_intPart, uint8_t *T_fractionalPart, bool *isPositive)
{
#ifdef USE_FAHRENHEIT
  float temperature_C = convertEncodedTemperatureToFloat(pAirData->T_C_int_with_sign,
                                                         pAirData->T_C_fr_1dp);
  convertCtoF_int(temperature_C, T_intPart, T_fractionalPart, isPositive);
  return FAHRENHEIT_SYMBOL;
#else
  isPositive[0] = ((pAirData->T_C_int_with_sign & TEMPERATURE_SIGN_MASK) == 0);
  T_intPart[0] = pAirData->T_C_int_with_sign & TEMPERATURE_VALUE_MASK;
  T_fractionalPart[0] = pAirData->T_C_fr_1dp;
  return CELSIUS_SYMBOL;
#endif
}

////////////////////////////////////////////////////////////////////////

// The following five functions print data (in floating-point
// representation) over the serial port as text

void printAirDataF(const AirData_F_t *airDataF)
{
  ESP_LOGI(TAG, "Temperature = ");
#ifdef USE_FAHRENHEIT
  float temperature_F = convertCtoF(airDataF->T_C);
  ESP_LOGI(TAG, "%f" FAHRENHEIT_SYMBOL "\n", temperature_F);
#else
  ESP_LOGI(TAG, "%f" CELSIUS_SYMBOL "\n", airDataF->T_C);
#endif
  ESP_LOGI(TAG, "Pressure = %d Pa\n", airDataF->P_Pa);
  ESP_LOGI(TAG, "Humidity = %f%%\n", airDataF->H_pc);
  ESP_LOGI(TAG, "Gas Sensor Resistance = %d " OHM_SYMBOL "\n", airDataF->G_Ohm);
}

void printAirQualityDataF(const AirQualityData_F_t *airQualityDataF)
{
  if (airQualityDataF->AQI_accuracy > 0)
  {
    ESP_LOGI(TAG, "Air Quality Index = %f (%s)\n", airQualityDataF->AQI, interpret_AQI_value((uint16_t)airQualityDataF->AQI));
    ESP_LOGI(TAG, "Estimated CO" SUBSCRIPT_2 " = %f ppm\n", airQualityDataF->CO2e);
    ESP_LOGI(TAG, "Equivalent Breath VOC = %f ppm\n", airQualityDataF->bVOC);
  }
  ESP_LOGI(TAG, "Air Quality Accuracy: %s", interpret_AQI_accuracy(airQualityDataF->AQI_accuracy));
}

void printLightDataF(const LightData_F_t *lightDataF)
{
  ESP_LOGI(TAG, "Illuminance = %f lux\n", lightDataF->illum_lux);
  ESP_LOGI(TAG, "White Light Level = %d%%\n", lightDataF->white);
}

void printSoundDataF(const SoundData_F_t *soundDataF)
{
  ESP_LOGI(TAG, "A-weighted Sound Pressure Level = %f dBA\n", soundDataF->SPL_dBA);
  for (uint16_t i = 0; i < SOUND_FREQ_BANDS; i++)
  {
    ESP_LOGI(TAG, "Frequency Band %u (%u Hz) SPL = %f dB\n", i + 1, sound_band_mids_Hz[i], soundDataF->SPL_bands_dB[i]);
  }
  ESP_LOGI(TAG, "Peak Sound Amplitude = %f mPa\n", soundDataF->peakAmp_mPa);
}

void printParticleDataF(const ParticleData_F_t *particleDataF, uint8_t particleSensor)
{
  ESP_LOGI(TAG, "Particle Duty Cycle = %f%%\n", particleDataF->duty_cycle_pc);
  if (particleSensor == PARTICLE_SENSOR_PPD42)
  {
    ESP_LOGI(TAG, "Particle Concentration = %f ppL\n", particleDataF->concentration);
  }
  else if (particleSensor == PARTICLE_SENSOR_SDS011)
  {
    ESP_LOGI(TAG, "Particle Concentration = %f " SDS011_UNIT_SYMBOL "\n", particleDataF->concentration);
  }
  else
  {
    ESP_LOGI(TAG, "Particle Concentration = %f (?)\n", particleDataF->concentration);
  }
  ESP_LOGI(TAG, "Particle data valid: ");
  if (particleDataF->valid)
  {
    ESP_LOGI(TAG, "Yes\n");
  }
  else
  {
    ESP_LOGI(TAG, "No (Initializing)\n");
  }
}

// ////////////////////////////////////////////////////////////////////////

// The following five functions print data (in integer representation) over the serial port as text.
// printColumns determines the print format:
// choosing printColumns = false gives labeled values with measurement units
// choosing printColumns = true gives columns of numbers (convenient for spreadsheets).

void printAirData(const AirData_t *airData, bool printColumns)
{
  uint8_t T_intPart = 0;
  uint8_t T_fractionalPart = 0;
  bool isPositive = true;
  const char *T_unit = getTemperature(airData, &T_intPart, &T_fractionalPart, &isPositive);

  if (printColumns)
  {
    // Print: temperature, pressure/Pa, humidity/%, gas sensor resistance/ohm
    ESP_LOGI(TAG, "%s%u.%u %" PRIu32 " %u.%u %" PRIu32 "\n", isPositive ? "" : "-", T_intPart, T_fractionalPart,
             airData->P_Pa, airData->H_pc_int, airData->H_pc_fr_1dp, airData->G_ohm);
  }
  else
  {
    ESP_LOGI(TAG, "Temperature = %s%u.%u %s\n", isPositive ? "" : "-", T_intPart, T_fractionalPart, T_unit);
    ESP_LOGI(TAG, "Pressure = %d Pa\n", airData->P_Pa);
    ESP_LOGI(TAG, "Humidity = %u.%u %%\n", airData->H_pc_int, airData->H_pc_fr_1dp);
    ESP_LOGI(TAG, "Gas Sensor Resistance = %d " OHM_SYMBOL "\n", airData->G_ohm);
  }
}

void printAirQualityData(const AirQualityData_t *airQualityData, bool printColumns)
{
  if (printColumns)
  {
    // Print: Air Quality Index, Estimated CO2/ppm, Equivalent breath VOC/ppm, Accuracy
    ESP_LOGI(TAG, "%u.%u %u.%u %u.%02u %u\n", airQualityData->AQI_int, airQualityData->AQI_fr_1dp,
             airQualityData->CO2e_int, airQualityData->CO2e_fr_1dp,
             airQualityData->bVOC_int, airQualityData->bVOC_fr_2dp, airQualityData->AQI_accuracy);
  }
  else
  {
    if (airQualityData->AQI_accuracy > 0)
    {
      ESP_LOGI(TAG, "Air Quality Index = %u.%u (%s)\n", airQualityData->AQI_int, airQualityData->AQI_fr_1dp, interpret_AQI_value(airQualityData->AQI_int));
      ESP_LOGI(TAG, "Estimated CO" SUBSCRIPT_2 " = %u.%u ppm\n", airQualityData->CO2e_int, airQualityData->CO2e_fr_1dp);
      ESP_LOGI(TAG, "Equivalent Breath VOC = %u.%02u ppm\n", airQualityData->bVOC_int, airQualityData->bVOC_fr_2dp);
    }
    ESP_LOGI(TAG, "Air Quality Accuracy: %s\n", interpret_AQI_accuracy(airQualityData->AQI_accuracy));
  }
}

void printSoundData(const SoundData_t *soundData, bool printColumns)
{
  if (printColumns)
  {
    // Print: Sound pressure level/dBA, Sound pressure level for frequency bands 1 to 6 (six columns),
    //        Peak sound amplitude/mPa, stability
    ESP_LOGI(TAG, "%u.%u\n", soundData->SPL_dBA_int, soundData->SPL_dBA_fr_1dp);
    for (uint16_t i = 0; i < SOUND_FREQ_BANDS; i++)
    {
      ESP_LOGI(TAG, "%u.%u\n", soundData->SPL_bands_dB_int[i], soundData->SPL_bands_dB_fr_1dp[i]);
    }
    ESP_LOGI(TAG, "%u.%02u %u\n", soundData->peak_amp_mPa_int, soundData->peak_amp_mPa_fr_2dp, soundData->stable);
  }
  else
  {
    ESP_LOGI(TAG, "A-weighted Sound Pressure Level = %u.%u dBA\n", soundData->SPL_dBA_int, soundData->SPL_dBA_fr_1dp);
    for (uint8_t i = 0; i < SOUND_FREQ_BANDS; i++)
    {
      ESP_LOGI(TAG, "Frequency Band %u (%u Hz) SPL = %u.%u dB", i + 1, sound_band_mids_Hz[i], soundData->SPL_bands_dB_int[i], soundData->SPL_bands_dB_fr_1dp[i]);
    }
    ESP_LOGI(TAG, "Peak Sound Amplitude = %u.%02u mPa", soundData->peak_amp_mPa_int, soundData->peak_amp_mPa_fr_2dp);
  }
}

void printLightData(const LightData_t *lightData, bool printColumns)
{
  if (printColumns)
  {
    // Print: illuminance/lux, white level
    ESP_LOGI(TAG, "%u.%02u %u\n", lightData->illum_lux_int, lightData->illum_lux_fr_2dp, lightData->white);
  }
  else
  {
    ESP_LOGI(TAG, "Illuminance = %u.%02u lux\n", lightData->illum_lux_int, lightData->illum_lux_fr_2dp);
    ESP_LOGI(TAG, "White Light Level = %d\n", lightData->white);
  }
}

void printParticleData(const ParticleData_t *particleData, bool printColumns, uint8_t particleSensor)
{
  if (printColumns)
  {
    // Print: duty cycle/%, concentration
    ESP_LOGI(TAG, "%u.%02u %u.%02u %u\n", particleData->duty_cycle_pc_int,
             particleData->duty_cycle_pc_fr_2dp, particleData->concentration_int,
             particleData->concentration_fr_2dp, particleData->valid);
  }
  else
  {
    ESP_LOGI(TAG, "Particle Duty Cycle = %u.%02u %%\n", particleData->duty_cycle_pc_int, particleData->duty_cycle_pc_fr_2dp);
    if (particleSensor == PARTICLE_SENSOR_PPD42)
    {
      ESP_LOGI(TAG, "Particle Concentration = %u.%02u ppL\n", particleData->concentration_int, particleData->concentration_fr_2dp);
    }
    else if (particleSensor == PARTICLE_SENSOR_SDS011)
    {
      ESP_LOGI(TAG, "Particle Concentration = %u.%02u " SDS011_UNIT_SYMBOL "\n", particleData->concentration_int, particleData->concentration_fr_2dp);
    }
    else
    {
      ESP_LOGI(TAG, "Particle Concentration = %u.%02u (?)\n", particleData->concentration_int, particleData->concentration_fr_2dp);
    }
    ESP_LOGI(TAG, "Particle data valid: ");
    if (particleData->valid == 0)
    {
      ESP_LOGI(TAG, "No (Initializing)\n");
    }
    else
    {
      ESP_LOGI(TAG, "Yes\n");
    }
  }
}
