const LOG_LEVEL = [ 'FATAL', 'ERROR', 'WARN', 'INFO', 'DEBUG' ];
const LOG_SOURCE = [ "ECU", "ESP", "CAN", "ADC", "TIM", "ACC", "LCD", "GPS" ];
const LOG_KEY = {
  'ECU': [
    { name: 'ECU_BOOT', parsed: null },
    { name: 'ECU_STATE', parsed: [ 'HV', 'RTD', 'BMS', 'IMD', 'BSPD', 'SD','CAN', 'ESP', 'ACC', 'LCD', 'GPS']  },
    { name: 'ECU_READY', parsed: null },
    { name: 'SD_INIT', parsed: null },
  ],
  'ESP': [
    { name: 'ESP_INIT', parsed: null },
    { name: 'ESP_REMOTE', parsed: null },
    { name: 'ESP_RTC_FIX', parsed: null },
  ],
  'CAN': [
    { name: 'CAN_INIT', parsed: null},
    { name: 'CAN_ERR', parsed: null},
    { name: 'CAN_INV_TEMP_1', parsed: [ 'igbt', 'gatedriver' ] },
    { name: 'CAN_INV_TEMP_2', parsed: [ 'controlboard', 'RTD1', 'RTD2', 'RTD3' ] },
    { name: 'CAN_INV_TEMP_3', parsed: [ 'coolant', 'hotspot', 'motor', 'torque_shudder' ] },
    { name: 'CAN_INV_ANALOG_IN', parsed: [ 'AIN1', 'AIN2', 'AIN3', 'AIN4','AIN5', 'AIN6' ]},
    { name: 'CAN_INV_DIGITAL_IN', parsed: [ 'DIN0', 'DIN1', 'DIN2', 'DIN3', 'DIN4', 'DIN5', 'DIN6', 'DIN7'] },
    { name: 'CAN_INV_MOTOR_POS', parsed: [ 'motor_angle', 'motor_speed', 'electrical_output_freq', 'delta_resolver_filtered' ] },
    { name: 'CAN_INV_CURRENT', parsed: [ 'phaseA', 'phaseB','phaseC','dc_bus_current' ] },
    { name: 'CAN_INV_VOLTAGE', parsed: [ 'dc_bus_voltage', 'output_voltage', 'VAB_Vd_voltage', 'VBC_Vq_voltage' ] },
    { name: 'CAN_INV_FLUX', parsed: [ 'flux_command', 'flux_feedback', 'Id_feedback', 'Iq_feedback' ] },
    { name: 'CAN_INV_REF', parsed: [ 'ref_1v5', 'ref_2v5', 'ref_5v', 'ref_12v' ] },
    { name: 'CAN_INV_STATE', parsed: [
      'vsm_state', 
      'pwm_freq', 
      'inverter_state', 
      'relay_state',
      'inverter_run_mode',
      'inverter_active_discharge_state',
      'inverter_command_mode',
      'inverter_enable_state',
      'inverter_start_mode_active',
      'inverter_enable_lockout',
      'direction_command',
      'bms_active',
      'bms_limiting_torque',
      'limit_max_speed',
      'limit_hot_spot',
      'low_speed_limiting',
      'coolant_temperature_limiting'
      ] },
    { name: 'CAN_INV_FAULT', parsed: [ 'POST', 'RUN', 'POST_FAULT_LO', 'POST_FAULT_HI','RUN_FAULT_LO','RUN_FAULT_HI' ] },
    { name: 'CAN_INV_TORQUE', parsed: [ 'commanded_torque', 'torque_feedback', 'power_on_timer']},
    { name: 'CAN_INV_FLUX_WEAKING', parsed: [ 'modulation_index', 'flux_weakening_output', 'Id_command', 'Iq_command' ] },
    { name: 'CAN_INV_FIRMWARE_VER', parsed: [ 'EEPROM_version', 'software_version', 'date_code', 'date_code_year' ] },
    { name: 'CAN_INV_DIAGNOSTIC', parsed: null },
    { name: 'CAN_INV_HIGH_SPD_MSG', parsed: null },
    { name: 'CAN_BMS_CORE', parsed: [ 'soc', 'capacity', 'voltage', 'current','failsafe' ] },
    { name: 'CAN_BMS_TEMP', parsed: [ 'temperature', 'dcl', 'ccl'] },
    { name: 'CAN_STEERING_WHEEL_ANGLE', parsed: ['angle', 'speed']}
    
  ], 
  'ADC': [
    { name: 'ADC_INIT', parsed: null},
    { name: 'ADC_CPU', parsed: [ 'CPU_TEMP', 'INPUT_VOLTAGE' ] },
    { name: 'ADC_DIST', parsed: [ 'DIST_FL', 'DIST_RL', 'DIST_FR', 'DIST_RR' ] },
    { name: 'ADC_A3', parsed: [ 'ADC_A3_FL', 'ADC_A3_RL', 'ADC_A3_FR', 'ADC_A3_RR' ] }
  ],
  'TIM': [
    { name: 'TIMER_IC', parsed: null}
  ],
  'ACC': [
    { name: 'ACC_INIT', parsed: null},
    { name: 'ACC_DATA', parsed: [ 'x', 'y', 'z' ] }
  ],
  'LCD': [
    { name: 'LCD_INIT', parsed: null},
    { name: 'LCD_UPDATED', parsed: null}

  ],
  'GPS': [
    { name: 'GPS_INIT', parsed: null},
    { name: 'GPS_POS', parsed: [ 'lat', 'lon' ] },
    { name: 'GPS_VEC', parsed: [ 'speed', 'course' ] },
    { name: 'GPS_TIME', parsed: [ 'utc_date', 'utc_time' ] }
  ]
};

function translate(raw) {
  try {
    let log = {
      timestamp: raw[0] + raw[1] * Math.pow(2, 8) + raw[2] * Math.pow(2, 16) + raw[3] * Math.pow(2, 24),
      datetime: null,
      level: LOG_LEVEL[raw[4]],
      source: LOG_SOURCE[raw[5]],
      key: LOG_SOURCE[raw[5]] === 'CAN' ? raw[6] : LOG_KEY[LOG_SOURCE[raw[5]]][raw[6]].name,
      checksum: raw[7] == ((raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5] + raw[6] + raw[8] + raw[9] + raw[10] + raw[11] + raw[12] + raw[13] + raw[14] + raw[15]) % 256),
      value: raw[8] + raw[9] * Math.pow(2, 8) + raw[10] * Math.pow(2, 16) + raw[11] * Math.pow(2, 24) + raw[12] * Math.pow(2, 32) + raw[13] * Math.pow(2, 40) + raw[14] * Math.pow(2, 48) + raw[15] * Math.pow(2, 56),
      raw: raw.slice(8),
      status: null
    }

    // validate checksum
    if (!log.checksum) {
      throw new Error('checksum error');
    }

    log.parsed = parse(log);
    log.status = convertToStatusFormat(log);
    return log;
  } catch(e) {
    console.error('Error in translate function:', e);
    return e;
  }
}

function convertToStatusFormat(log) {
  const status = {
    ESP: log.raw[0] & 0x01,
    car: {
      system: {
        ESP: log.raw[0] & 0x01,
        HV: log.raw[0] & 0x02,
        RTD: log.raw[0] & 0x04,
        SD: log.raw[0] & 0x08,
        CAN: log.raw[0] & 0x10,
        ACC: log.raw[0] & 0x20,
        LCD: log.raw[0] & 0x40,
        GPS: log.raw[0] & 0x80,
        IMD: log.raw[1] & 0x01,
        BMS: log.raw[1] & 0x02,
        BSPD: log.raw[1] & 0x04,
      },
      speed: log.value & 0xFFFF,
      rpm: (log.value >> 16) & 0xFFFF,
      torque: (log.value >> 32) & 0xFFFF,
    },
    temperature: log.raw[2],
    bms: {
      failsafe: {
        voltage: log.raw[3] & 0x01,
        current: log.raw[3] & 0x02,
        relay: log.raw[3] & 0x04,
        balancing: log.raw[3] & 0x08,
        interlock: log.raw[3] & 0x10,
        thermistor: log.raw[3] & 0x20,
        power: log.raw[3] & 0x40,
      },
      soc: log.raw[4],
      capacity: log.raw[5],
      voltage: log.raw[6],
      current: log.raw[7],
      dcl: log.raw[8],
      temperature: {
        max: {
          temp: log.raw[9],
          id: log.raw[10]
        }
      }
    }
  };

  return status;
}

function parse(log) {
  let parsed;

  let source = log.source;
  let key = log.key;
  let raw = log.raw;

  switch (source) {
    case 'ECU': {
      switch (key) {
        case "ECU_BOOT":
        case "ECU_STATE": {
          parsed = {
            HV: value & 1 << 0 ? true : false,
            RTD: value & 1 << 1 ? true : false,
            BMS: value & 1 << 2 ? true : false,
            IMD: value & 1 << 3 ? true : false,
            BSPD: value & 1 << 4 ? true : false,

            SD: value & 1 << 5 ? true : false,
            CAN: value & 1 << 6 ? true : false,
            ESP: value & 1 << 7 ? true : false,
            ACC: value & 1 << 8 ? true : false,
            LCD: value & 1 << 9 ? true : false,
            GPS: value & 1 << 10 ? true : false
          };
          break;
        }

        case "ECU_READY": 
        case "SD_INIT": 
        default: {
          parsed = null;
          break;
        }
      }
      break;

    }
    case 'ESP':
    case 'CAN': {
      switch (key) {
        case "CAN_INIT": 
          break;
        case "CAN_ERR":
          break; 
        case "CAN_INV_TEMP_1": {
          parsed = {
            igbt: {
              a: signed(value & 0xffff, 16) * 0.1,
              b: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
              c: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            },
            gatedriver: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          parsed.igbt.max = parsed.igbt.a > parsed.igbt.b ? (parsed.igbt.a > parsed.igbt.c ? { temperature: parsed.igbt.a, id: "A" } : { temperature: parsed.igbt.c, id: "C" }) : (parsed.igbt.b > parsed.igbt.c ? { temperature: parsed.igbt.b, id: "B" } : { temperature: parsed.igbt.c, id: "C" });
          break;
        }

        case "CAN_INV_TEMP_2": {
          parsed = {
            controlboard: signed(raw[0] + raw[1] * Math.pow(2, 8), 16) * 0.1,
            RTD1: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            RTD2: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            RTD3: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_TEMP_3": {
          parsed = {
            coolant: signed(value & 0xffff, 16) * 0.1,
            hotspot: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            motor: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            torque_shudder: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_ANALOG_IN": {
          parsed = {
            AIN1: signed((raw[0] + raw[1] * Math.pow(2, 8)) & 0x3ff, 10) * 0.01,
            AIN2: signed((value / Math.pow(2, 10)) & 0x3ff, 10) * 0.01,
            AIN3: signed((value / Math.pow(2, 20)) & 0x3ff, 10) * 0.01,
            AIN4: signed((value / Math.pow(2, 30)) & 0x3ff, 10) * 0.01,
            AIN5: signed((value / Math.pow(2, 40)) & 0x3ff, 10) * 0.01,
            AIN6: signed((value / Math.pow(2, 50)) & 0x3ff, 10) * 0.01,
          };
          break;
        }

        case "CAN_INV_DIGITAL_IN": {
          parsed = {
            DIN1: (value & 0xff) ? true : false,
            DIN2: ((value / Math.pow(2, 8)) & 0xff) ? true : false,
            DIN3: ((value / Math.pow(2, 16)) & 0xff) ? true : false,
            DIN4: ((value / Math.pow(2, 24)) & 0xff) ? true : false,
            DIN5: ((value / Math.pow(2, 32)) & 0xff) ? true : false,
            DIN6: ((value / Math.pow(2, 40)) & 0xff) ? true : false,
            DIN7: ((value / Math.pow(2, 48)) & 0xff) ? true : false,
            DIN8: ((value / Math.pow(2, 56)) & 0xff) ? true : false,
          };
          break;
        }

        case "CAN_INV_MOTOR_POS": {
          parsed = {
            motor_angle: signed(value & 0xffff, 16) * 0.1,
            motor_speed: signed((value / Math.pow(2, 16)) & 0xffff, 16),
            electrical_output_freq: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            delta_resolver_filtered: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_CURRENT": {
          parsed = {
            phaseA: signed(value & 0xffff, 16) * 0.1,
            phaseB: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            phaseC: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            dc_bus_current: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_VOLTAGE": {
          parsed = {
            dc_bus_voltage: signed(value & 0xffff, 16) * 0.1,
            output_voltage: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            VAB_Vd_voltage: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            VBC_Vq_voltage: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_FLUX": {
          parsed = {
            flux_command: signed(value & 0xffff, 16) * 0.001,
            flux_feedback: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.001,
            Id_feedback: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            Iq_feedback: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_REF": {
          parsed = {
            ref_1v5: signed(value & 0xffff, 16) * 0.01,
            ref_2v5: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.01,
            ref_5v: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.01,
            ref_12v: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.01,
          };
          break;
        }

        case "CAN_INV_STATE": {
          parsed = {
            vsm_state: raw[0],
            pwm_freq: raw[1],
            inverter_state: raw[2],
            relay_state:raw[3],
            inverter_run_mode: raw[4] & 0x1,
            inverter_active_discharge_state: (raw[4] / Math.pow(2, 5)) & 0b111,
            inverter_command_mode: raw[5] & 0x1,
            inverter_enable_state: raw[6] & 0x1,
            inverter_start_mode_active: (raw[6] / Math.pow(2, 6)) & 0x1,
            inverter_enable_lockout: (raw[6] / Math.pow(2, 7)) & 0x1,
            direction_command: raw[7] & 0x1,
            bms_active: (raw[7] / Math.pow(2, 1)) & 0x1,
            bms_limiting_torque: (raw[7] / Math.pow(2, 2)) & 0x1,
            limit_max_speed: (raw[7] / Math.pow(2, 3)) & 0x1,
            limit_hot_spot: (raw[7] / Math.pow(2, 4)) & 0x1,
            low_speed_limiting: (raw[7] / Math.pow(2, 5)) & 0x1,
            coolant_temperature_limiting: (raw[7] / Math.pow(2, 6)) & 0x1,
          };
          break;
        }

        case "CAN_INV_FAULT": {
          parsed = {
            POST: value & 0xffffffff,
            RUN: (value / Math.pow(2, 32)) & 0xffffffff,
            POST_FAULT_LO: value & 0xffff,
            POST_FAULT_HI: (value / Math.pow(2, 16)) & 0xffff,
            RUN_FAULT_LO: (value / Math.pow(2, 32)) & 0xffff,
            RUN_FAULT_HI: (value / Math.pow(2, 48)) & 0xffff,
          };
          break;
        }

        case "CAN_INV_TORQUE": {
          parsed = {
            commanded_torque: signed(value & 0xffff, 16) * 0.1,
            torque_feedback: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            power_on_timer: value / Math.pow(2, 32)
          };
          break;
        }

        case "CAN_INV_FLUX_WEAKING": {
          parsed = {
            modulation_index: (value & 0xffff) * 0.01,
            flux_weakening_output: signed((value / Math.pow(2, 16)) & 0xffff, 16) * 0.1,
            Id_command: signed((value / Math.pow(2, 32)) & 0xffff, 16) * 0.1,
            Iq_command: signed((value / Math.pow(2, 48)) & 0xffff, 16) * 0.1,
          };
          break;
        }

        case "CAN_INV_FIRMWARE_VER": {
          parsed = {
            EEPROM_version: value & 0xffff,
            software_version: (value / Math.pow(2, 16)) & 0xfff,
            date_code: (value / Math.pow(2, 32)) & 0xffff,
            date_code_year: (value / Math.pow(2, 48)) & 0xffff,
          };
          break;
        }

        case "CAN_INV_DIAGNOSTIC": {
          parsed = null;
          break;
        }

        case "CAN_INV_HIGH_SPD_MSG": {
          parsed = null;
          break;
        }

        case "CAN_BMS_CORE": {
          const failsafe = raw[7] + raw[6] * Math.pow(2, 8);
          parsed = {
            soc: raw[0] * 0.5,
            capacity: raw[1] * 0.1,
            voltage: (raw[3] + raw[2] * Math.pow(2, 8)) * 0.1,
            current: signed(raw[5] + raw[4] * Math.pow(2, 8), 16) * 0.1,
            failsafe: {
              voltage: failsafe & 1 << 0 ? true : false,
              current: failsafe & 1 << 1 ? true : false,
              relay: failsafe & 1 << 2 ? true : false,
              balancing: failsafe & 1 << 3 ? true : false,
              interlock: failsafe & 1 << 4 ? true : false,
              thermistor: failsafe & 1 << 5 ? true : false,
              power: failsafe & 1 << 6 ? true : false,
            }
          };
          break;
        }

        case "CAN_BMS_TEMP": {
          parsed = {
            temperature: {
              max: {
                value: signed(raw[0], 8),
                id: raw[1],
              },
              min: {
                value: signed(raw[2], 8),
                id: raw[3],
              },
            },
            dcl: (raw[5] + raw[4] * Math.pow(2, 8)),
            ccl: (raw[7] + raw[6] * Math.pow(2, 8)),
          };
          break;
        }

        case "CAN_STEERING_WHEEL_ANGLE": {
          parsed = {
            angle: signed(raw[0] + raw[1] * Math.pow(2, 8), 16) * 0.1, // ±780° 범위
            speed: signed(raw[2] + raw[3] * Math.pow(2, 8), 16) * 4 // 0 to 1,016°/s 범위
          };
          break;
        }
        

        default: {
          parsed = null;
          break;
        }
      }
      break;
    }

    case 'ADC': {
      const resolution = 12; // ADC resolution in bits
      const max = (1 << resolution) - 1;

      switch (key) {
        case "ADC_INIT":{
          parsed = null;
          break;
        }
        case "ADC_CPU": {
          parsed = {
            CPU_TEMP: (raw[0] + raw[1] * Math.pow(2, 8)) / 10,
            INPUT_VOLTAGE: (raw[2] + raw[3] * Math.pow(2, 8)) / max * 3.3 * 8,
          };
          break;
        }
        case 'ADC_DIST': {
          function calc_position(value) {
            const adc_volatage = 3.3;
            const adc_resolution = 12;
            const adc_max_count = (1 << 12) - 1;
            const sensor_max_travel = 100; // mm
            const sensor_voltage = 5;

            const voltage = value / adc_max_count * adc_volatage;
            const dist = voltage / sensor_voltage * sensor_max_travel;

            return dist.toFixed(1);
          }
          parsed = {
            DIST_FL: calc_position(raw[0] + raw[1] * Math.pow(2, 8)),
            DIST_RL: calc_position(raw[2] + raw[3] * Math.pow(2, 8)),
            DIST_FR: calc_position(raw[4] + raw[5] * Math.pow(2, 8)),
            DIST_RR: calc_position(raw[6] + raw[7] * Math.pow(2, 8)),
          };
          break;
        }

        case 'ADC_A3': {
          function adc_to_voltage(value) {
            const adc_volatage = 3.3;
            const adc_resolution = 12;
            const adc_max_count = (1 << 12) - 1;
  
            const voltage = value / adc_max_count * adc_volatage;
                
            return voltage.toFixed(3);
          }
          parsed = {
            ADC_A3_FL: adc_to_voltage(raw[0] + raw[1] * Math.pow(2, 8)),
            ADC_A3_RL: adc_to_voltage(raw[2] + raw[3] * Math.pow(2, 8)),
            ADC_A3_FR: adc_to_voltage(raw[4] + raw[5] * Math.pow(2, 8)),
            ADC_A3_RR: adc_to_voltage(raw[6] + raw[7] * Math.pow(2, 8)),
          };
          break;
        }

        default:
          parsed = null;
          break;
      }
      break;
    } // case 'ADC'
    case 'TIM':
    case 'ACC': {
      switch (key) {
        case "ACC_INIT":{
          parsed = null;
          break;
        }
        case 'ACC_DATA': {
          parsed = {
            x: 4 / 512 * signed(raw[0] + raw[1] * Math.pow(2, 8), 16),
            y: 4 / 512 * signed(raw[2] + raw[3] * Math.pow(2, 8), 16),
            z: 4 / 512 * signed(raw[4] + raw[5] * Math.pow(2, 8), 16),
          };
          break;
        }

        default:
          parsed = null;
          break;
      }
      break;
    } // case 'ACC'
    case 'LCD': 
      
    case 'GPS': {
      switch (key) {
        case "GPS_INIT":
        case 'GPS_POS': {
          const raw_lat = (raw[0] + raw[1] * Math.pow(2, 8) + raw[2] * Math.pow(2, 16) + raw[3] * Math.pow(2, 24)) * 0.0000001;
          const raw_lon = (raw[4] + raw[5] * Math.pow(2, 8) + raw[6] * Math.pow(2, 16) + raw[7] * Math.pow(2, 24)) * 0.0000001;

          parsed = {
            lat: Math.floor(raw_lat) + (((raw_lat % 1) * 100).toFixed(5) / 60),
            lon: Math.floor(raw_lon) + (((raw_lon % 1) * 100).toFixed(5) / 60),
          };
          break;
        }

        case 'GPS_VEC': {
          parsed = {
            speed: (raw[0] + raw[1] * Math.pow(2, 8) + raw[2] * Math.pow(2, 16) + raw[3] * Math.pow(2, 24)) * 0.01 * 1.852,
            course: (raw[4] + raw[5] * Math.pow(2, 8) + raw[6] * Math.pow(2, 16) + raw[7] * Math.pow(2, 24))
          };
          break;
        }

        case 'GPS_TIME': {
          parsed = {
            utc_date: (raw[0] + raw[1] * Math.pow(2, 8) + raw[2] * Math.pow(2, 16) + raw[3] * Math.pow(2, 24)),
            utc_time: (raw[4] + raw[5] * Math.pow(2, 8) + raw[6] * Math.pow(2, 16) + raw[7] * Math.pow(2, 24))
          };
          break;
        }
        

        default:
          parsed = null;
          break;
      }
      break;
    } // case 'GPS'
  } // switch (source)

  return parsed;
}

function parse_CAN(source, raw) {
  switch (source.type) {
    case 'byte': {
      let value = 0;

      switch (source.info.endian) {
        case 'big': {
          for (let i = Number(source.info.end), cnt = 0; i > Number(source.info.start) - 1; i--, cnt++) {
            value += raw[i] * Math.pow(2, cnt * 8);
          }
          return value;
        }

        case 'little': {
          for (let i = Number(source.info.start), cnt = 0; i < Number(source.info.end) + 1; i++, cnt++) {
            value += raw[i] * Math.pow(2, cnt * 8);
          }
          return value;
        }

        default:
          return null;
      }
      break;
    }

    case 'bit': {
      let including_byte = {
        start: Math.floor(Number(source.info.start) / 8),
        end: Math.floor(Number(source.info.end) / 8)
      };

      let offset = Number(source.info.start) - 8 * including_byte.start;
      // TODO
      break;
    }
  }
}

function signed(value, bit) {
  return value > Math.pow(2, bit - 1) - 1 ? value - Math.pow(2, bit) : value;
}

exports.LOG_LEVEL = LOG_LEVEL;
exports.LOG_SOURCE = LOG_SOURCE;
exports.LOG_KEY= LOG_KEY;
exports.translate = translate;