#include "L3GD20.h"
#include "setila_errors.h"

int L3GD20::apply_config() {
  return set_mode_of_operation(config.output_data_rate, config.full_scale,
                               config.mode_of_operation, config.fifo_type);
}

int L3GD20::set_mode_of_operation(
    ST_Sensor::OUTPUT_DATA_RATE output_data_rate,
    ST_Sensor::FULL_SCALE full_scale,
    ST_Sensor::MODE_OF_OPERATION mode_of_operation,
    ST_Sensor::FIFO_TYPE fifo_type) {
  if (!m_device_id_verified) {
    if (verify_device_id(L3GD20::REG_WHO_AM_I, L3GD20::DEVICE_ID)) {
      return ERROR_WRONG_DEVICE_MODEL;
    } else {
      m_device_id_verified = true;
    }
  }

  config.output_data_rate = output_data_rate;
  config.full_scale = full_scale;
  config.fifo_type = fifo_type;
  config.mode_of_operation = mode_of_operation;

  switch (mode_of_operation) {
  case ST_Sensor::MODE_OF_OPERATION::OP_NORMAL_MODE:
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_PD_BIT);
    break;
  case ST_Sensor::MODE_OF_OPERATION::OP_POWER_DOWN:
    m_CTRL_REG1 &= (~(1 << L3GD20::CTRL_REG1_PD_BIT));
    break;
  case ST_Sensor::MODE_OF_OPERATION::OP_SLEEP_MODE:
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_PD_BIT);
    m_CTRL_REG1 &=
        (~((1 << L3GD20::CTRL_REG1_Xen_BIT) | (1 << L3GD20::CTRL_REG1_Yen_BIT) |
           (1 << L3GD20::CTRL_REG1_Zen_BIT)));
    break;
  default:
    return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
  }

  // Do not overwrite output registers while reading is still in progress
  m_CTRL_REG4 |= (1 << L3GD20::CTRL_REG4_BDU_BIT);

  // Define the output data rate
  switch (output_data_rate) {
  case ST_Sensor::OUTPUT_DATA_RATE::ODR_95_Hz:
    m_CTRL_REG1 &= (~((1 << L3GD20::CTRL_REG1_DR1_BIT) |
                      (1 << L3GD20::CTRL_REG1_DR0_BIT)));
    break;
  case ST_Sensor::OUTPUT_DATA_RATE::ODR_190_Hz:
    m_CTRL_REG1 &= (~(1 << L3GD20::CTRL_REG1_DR1_BIT));
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_DR0_BIT);
    break;
  case ST_Sensor::OUTPUT_DATA_RATE::ODR_380_Hz:
    m_CTRL_REG1 &= (~(1 << L3GD20::CTRL_REG1_DR0_BIT));
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_DR1_BIT);
    break;
  case ST_Sensor::OUTPUT_DATA_RATE::ODR_760_Hz:
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_DR1_BIT);
    m_CTRL_REG1 |= (1 << L3GD20::CTRL_REG1_DR0_BIT);
    break;
  default:
    return ERROR_UNSUPPORTED_DEVICE_OPTION_CONFIG;
  }

  if (fifo_type != ST_Sensor::FIFO_TYPE::FIFO_DISABLED) {
    // Reset the FIFO type by selecting bypass mode
    m_FIFO_CTRL_REG &= (~((1 << L3GD20::FIFO_CTRL_REG_FM2_BIT) |
                          (1 << L3GD20::FIFO_CTRL_REG_FM1_BIT) |
                          (1 << L3GD20::FIFO_CTRL_REG_FM0_BIT)));
    // FIFO enable
    m_CTRL_REG5 |= (1 << L3GD20::CTRL_REG5_FIFO_EN_BIT);
  } else {
    // Disable FIFO
    m_CTRL_REG5 &= (~(1 << L3GD20::CTRL_REG5_FIFO_EN_BIT));
  }

  // Write the registers in the order given in L3GD20 application note
  if (m_interface->write(L3GD20::REG_CTRL_REG4, &m_CTRL_REG4, 1)) {
    return ERROR_WRITE_FAILED;
  };

  if (m_interface->write(L3GD20::REG_CTRL_REG5, &m_CTRL_REG5, 1)) {
    return ERROR_WRITE_FAILED;
  }

  if (m_interface->write(L3GD20::REG_CTRL_REG1, &m_CTRL_REG1, 1)) {
    return ERROR_WRITE_FAILED;
  }

  // Set FIFO in bypass mode
  if (m_interface->write(L3GD20::REG_FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1)) {
    return ERROR_WRITE_FAILED;
  }

  return 0;
}

int L3GD20::get_sensor_readings() {
  if (!m_device_id_verified) {
    if (verify_device_id(L3GD20::REG_WHO_AM_I, L3GD20::DEVICE_ID)) {
      return ERROR_WRONG_DEVICE_MODEL;
    } else {
      m_device_id_verified = true;
    }
  }

  int status = 0;
  bool fifo_active = true;

  switch (config.fifo_type) {
  case ST_Sensor::FIFO_TYPE::FIFO_DISABLED:
    status = get_data_registers();
    fifo_active = false;
    break;
  case ST_Sensor::FIFO_TYPE::BYPASS_MODE:
    status = get_data_registers_bypass_mode();
    break;
  case ST_Sensor::FIFO_TYPE::FIFO:
    status = get_data_registers_fifo_mode();
    break;
  case ST_Sensor::FIFO_TYPE::STREAM_TO_FIFO:
    status = get_data_registers_stream_to_fifo_mode();
    break;
  case ST_Sensor::FIFO_TYPE::STREAM:
    status = get_data_registers_stream_mode();
    break;
  case ST_Sensor::FIFO_TYPE::BYPASS_TO_FIFO:
    status = get_data_registers_bypass_to_fifo_mode();
    break;
  default:
    fifo_active = false;
    break;
  }

  if (fifo_active) {
    // Back to bypass mode before using FIFO again
    reset_FIFO_to_bypass_mode();
  }

  return status;
}

int L3GD20::get_data_registers_fifo_mode() {
  int status = 0;

  // Select FIFO mode in FIFO control register
  m_FIFO_CTRL_REG &= (~((1 << L3GD20::FIFO_CTRL_REG_FM2_BIT) |
                        (1 << L3GD20::FIFO_CTRL_REG_FM1_BIT)));
  m_FIFO_CTRL_REG |= (1 << L3GD20::FIFO_CTRL_REG_FM0_BIT);
  // Activate FIFO mode to start the measurement
  if (m_interface->write(L3GD20::REG_FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1)) {
    return ERROR_WRITE_FAILED;
  }

  uint8_t fifo_src_reg = 0;
  int retries_counter = 65536;

  // Wait till the FIFO is full (overrun bit set to 1)
  do {
    if (m_interface->read(L3GD20::REG_FIFO_SRC_REG, &fifo_src_reg, 1)) {
      return ERROR_READ_FAILED;
    }
    retries_counter--;
    if (retries_counter == 0) {
      return ERROR_SENSOR_READING_TIME_OUT;
    }
  } while (!(fifo_src_reg & (1 << L3GD20::FIFO_SRC_OVRN_BIT)));

  // Read the whole FIFO with one read command using auto increment address (MSB
  // of the start address set to 1)
  if (m_interface->read((L3GD20::REG_OUT_X_L | 0x80), data.FIFO,
                        L3GD20::FIFO_SIZE_IN_BYTES)) {
    status = ERROR_READ_FAILED;
  }

  return status;
}

int L3GD20::get_data_registers() {
  int status = 0;

  uint8_t status_reg = 0;
  uint8_t retries_counter = 255;

  // Wait till the data are available (overrun bit set to 1)
  do {
    if (m_interface->read(L3GD20::REG_STATUS_REG, &status_reg, 1)) {
      return ERROR_READ_FAILED;
    }
    retries_counter--;
    if (retries_counter == 0) {
      return ERROR_SENSOR_READING_TIME_OUT;
    }
  } while (!(status_reg & (1 << L3GD20::STATUS_REG_ZYXDA_BIT)));

  // Read the whole FIFO with one read command using auto increment address (MSB
  // of the start address set to 1)
  if (m_interface->read((L3GD20::REG_OUT_X_L | 0x80),
                        &(data.FIFO[L3GD20::FIFO_SIZE_IN_RAW_VALUES - 3]), 6)) {
    status = ERROR_READ_FAILED;
  }

  return status;
}

int L3GD20::get_data_registers_stream_to_fifo_mode() {
  // TODO implement data reading
#if 0
	// Select STREAN_TO_FIFO mode in FIFO control register
	m_FIFO_CTRL_REG &= (~(1 << L3GD20::FIFO_CTRL_REG_FM2));
	m_FIFO_CTRL_REG |= ((1 << L3GD20::FIFO_CTRL_REG_FM1) | ((1 << L3GD20::FIFO_CTRL_REG_FM0)));

	// Activate FIFO mode to start the measurement
	if (m_interface->write(L3GD20::FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1)) {
		return ERROR_WRITE_FAILED;
	}

#endif

  return 0;
}

int L3GD20::get_data_registers_stream_mode() {
  // TODO implement data reading
#if 0
	// Select STREAM mode in FIFO control register
	m_FIFO_CTRL_REG &= (~((1 << L3GD20::FIFO_CTRL_REG_FM2) | (1 << L3GD20::FIFO_CTRL_REG_FM0)));
	m_FIFO_CTRL_REG |= (1 << L3GD20::FIFO_CTRL_REG_FM1);
	// Activate STREAM mode to start the measurement
	if (m_interface->write(L3GD20::FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1)) {
		return ERROR_WRITE_FAILED;
	}

#endif
  return 0;
}

int L3GD20::get_data_registers_bypass_to_fifo_mode() {
  // TODO implement data reading
#if 0
	// Select FIFO mode in FIFO control register
	m_FIFO_CTRL_REG &= (~((1 << L3GD20::FIFO_CTRL_REG_FM1) | (1 << L3GD20::FIFO_CTRL_REG_FM0)));
	m_FIFO_CTRL_REG |= (1 << L3GD20::FIFO_CTRL_REG_FM2);

	// Activate FIFO mode to start the measurement
	if (m_interface->write(L3GD20::FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1)) {
		return ERROR_WRITE_FAILED;
	}

#endif

  return 0;
}

int L3GD20::get_data_registers_bypass_mode() {
  // TODO implement data reading
  return 0;
}

int L3GD20::reset_FIFO_to_bypass_mode() {
  m_FIFO_CTRL_REG &= (~((1 << L3GD20::FIFO_CTRL_REG_FM2_BIT) |
                        (1 << L3GD20::FIFO_CTRL_REG_FM1_BIT) |
                        (1 << L3GD20::FIFO_CTRL_REG_FM0_BIT)));
  return m_interface->write(L3GD20::REG_FIFO_CTRL_REG, &m_FIFO_CTRL_REG, 1);
}

double L3GD20::angular_rate_in_mdps(const L3GD20::Config &conf,
                                    int16_t raw_angular_rate) {
  switch (conf.full_scale) {
  case ST_Sensor::FULL_SCALE::FS_250_DPS:
    return raw_angular_rate * 0.00875;
    break;
  case ST_Sensor::FULL_SCALE::FS_500_DPS:
    return raw_angular_rate * 0.01750;
    break;
  case ST_Sensor::FULL_SCALE::FS_2000_DPS:
    return raw_angular_rate * 0.070;
    break;
  }

  return 0.;
}

int L3GD20::set_mode_of_operation(
    ST_Sensor::MODE_OF_OPERATION mode_of_operation,
    ST_Sensor::OUTPUT_DATA_RATE output_data_rate) {
  return 0;
}

int L3GD20::set_resolution(uint8_t average_1, uint8_t average_2) { return 0; }
