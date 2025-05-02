use crate::estimation::types::EstimationBus;
use crate::{fsw_types::ParamBus, sensors::types::*};
use altai_rs::{quatlib::qxform, types::*};

#[derive(Debug, Default, Clone, Copy)]
pub struct RawIMUPacket {
    // Timestamped data coming directly from IMU in IMU frame
    // Meta
    raw_timestamp: u32,
    raw_valid: bool,
    msg_counter: u32,

    // Sensor Specific
    raw_gyro: [f64; 3],  // Rates in IMU frame
    raw_accel: [f64; 3], // Accel in IMU frame
}
impl RawSensorPacket for RawIMUPacket {}

impl RawIMUPacket {
    pub fn plant_update(
        &mut self,
        timestamp: u32,
        raw_valid: bool,
        inc_msg: bool,
        raw_gyro: [f64; 3],
        raw_accel: [f64; 3],
    ) {
        self.raw_timestamp = timestamp;
        self.raw_valid = raw_valid;
        self.msg_counter += inc_msg as u32;
        self.raw_gyro = raw_gyro;
        self.raw_accel = raw_accel;
    }
}

#[derive(Debug, Clone)]
pub struct SensProcIMUBus {
    // Processed data coming off IMU
    // Meta
    timestamp: u32,
    error_code: u16,
    n_imu: usize,
    prev_msg_counter: u32,

    // Sensor Specific
    gyro_sc: Vector3,  // Rates in SC frame
    accel_sc: Vector3, // Accel in SC frame
}

impl Sensor for SensProcIMUBus {
    type Packet = RawIMUPacket;
    fn process(
        &mut self,
        packets: &[Self::Packet],
        _prev_estimation_bus: &EstimationBus,
        param_bus: &ParamBus,
    ) {
        // Reset
        self.error_code = 0u16;

        // Check Enabled
        let enabled = true; // TODO -> External Check
        self.update_hw_test(enabled, 0); // HW Valid if Enabled

        // Check Message Counter
        let msg_inc = packets.iter().fold(true, |flag, imu| {
            flag & (imu.msg_counter != self.prev_msg_counter)
        });
        self.prev_msg_counter = packets[0].msg_counter;
        self.update_hw_test(msg_inc, 1); // HW Valid if MSG Counter Incrementing

        // Check Raw Valid
        let valid = packets
            .iter()
            .fold(0, |acc, imu| acc + imu.raw_valid as usize)
            > self.n_imu / 2;
        self.update_hw_test(valid, 2); // Valid if >half IMU is valid

        // Check timestamp staleness
        self.timestamp =
            packets.iter().fold(0, |acc, imu| acc + imu.raw_timestamp) / self.n_imu as u32;
        let valid = packets.iter().fold(true, |acc, imu| {
            acc | ((self.timestamp as i32 - imu.raw_timestamp as i32).abs() < 10)
        });
        self.update_hw_test(valid, 3); // Valid if each timestamp within 1 sec of average

        // Update Data
        self.ingest(packets, param_bus);
    }

    fn ingest(&mut self, packets: &[Self::Packet], param_bus: &ParamBus) {
        // Transform to SC frame
        let tfr_gyro = qxform(
            &param_bus.acs_sensors.q_sc_imu,
            &Vector3::from_shape_fn((3, self.n_imu), |(row, col)| packets[col].raw_gyro[row]),
        );

        let tfr_accl = qxform(
            &param_bus.acs_sensors.q_sc_imu,
            &Vector3::from_shape_fn((3, self.n_imu), |(row, col)| packets[col].raw_accel[row]),
        );

        // Move to Self
        self.gyro_sc.assign(&tfr_gyro);
        self.accel_sc.assign(&tfr_accl);
    }

    fn hardware_subtest(&self) -> u16 {
        /* MSB
        15
        14
        13
        12
        11
        10
        09
        08
        07
        06
        05
        04
        03: All IMU Timestamp < 1 sec from average
        02: >n/2 IMU Valid
        01: MsgCounter Increasing
        00: Enabled
        LSB */
        self.error_code
    }
}

impl SensProcIMUBus {
    pub fn initialize(n_imu: usize) -> Self {
        Self {
            // Meta
            timestamp: 0,
            error_code: 0u16,
            n_imu,
            prev_msg_counter: 0u32,

            // Sensor-Specific
            gyro_sc: Vector3::zeros((3, n_imu)),
            accel_sc: Vector3::zeros((3, n_imu)),
        }
    }

    fn update_hw_test(&mut self, flag: bool, bit_id: u8) {
        if bit_id > 15 {
            panic!("Invalid bit setting for u16 bitpack")
        }
        self.error_code ^= (0b1 * !flag as u16) << bit_id;
    }
}

impl Default for SensProcIMUBus {
    fn default() -> Self {
        let n_imu = 1;
        Self::initialize(n_imu)
    }
}
