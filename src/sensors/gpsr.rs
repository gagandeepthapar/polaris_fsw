use crate::{fsw_types::ParamBus, sensors::types::*};
use altai_rs::types::*;

#[derive(Debug, Default, Clone, Copy)]
pub struct RawGPSRPacket {
    // Timestamped attitude coming direct from startracker
    // Meta
    raw_timestamp: u32,
    raw_valid: bool,
    msg_counter: u32,

    // Sensor-Specific Data
    raw_r_eci: [f64; 3], // TODO: REPLACE w R_ECEF
    raw_v_eci: [f64; 3], // TODO: REPLACE w V_ECEF
}
impl RawSensorPacket for RawGPSRPacket {}

impl RawGPSRPacket {
    pub fn plant_update(
        &mut self,
        timestamp: u32,
        raw_valid: bool,
        inc_msg: bool,
        raw_r_eci: [f64; 3],
        raw_v_eci: [f64; 3],
    ) {
        self.raw_timestamp = timestamp;
        self.raw_valid = raw_valid;
        self.msg_counter += inc_msg as u32;
        self.raw_r_eci = raw_r_eci;
        self.raw_v_eci = raw_v_eci;
    }
}

#[derive(Debug, Clone)]
pub struct SensProcGPSRBus {
    // Processed data coming off STA
    // Meta
    timestamp: u32,
    error_code: u16,
    n_gpsr: usize,
    prev_msg_counter: u32,

    // Sensor-Specific
    r_eci: Vector3, // SV Position in ECI // TODO: REPLACE w/ R_ECEF
    v_eci: Vector3, // SV Velocity in ECI // TODO: REPLACE w/ R_ECEF
}

impl Sensor for SensProcGPSRBus {
    type Packet = RawGPSRPacket;
    fn process(
        &mut self,
        packets: &[Self::Packet],
        _prev_estimation_bus: &crate::estimation::types::EstimationBus,
        param_bus: &ParamBus,
    ) {
        // Reset
        self.error_code = 0u16;

        // Check Enabled
        let enabled = true; // TODO -> External Check
        self.update_hw_test(enabled, 0); // HW Valid if Enabled

        // Check Message Counter
        let msg_inc = packets.iter().fold(true, |flag, gps| {
            flag & (gps.msg_counter != self.prev_msg_counter)
        });
        self.prev_msg_counter = packets[0].msg_counter;
        self.update_hw_test(msg_inc, 1); // HW Valid if MSG Counter Incrementing

        // Check Raw Valid
        let valid = packets
            .iter()
            .fold(0, |acc, gps| acc + gps.raw_valid as usize)
            > self.n_gpsr / 2;
        self.update_hw_test(valid, 2); // Valid if >half STA is valid

        // Check timestamp staleness
        self.timestamp =
            packets.iter().fold(0, |acc, gps| acc + gps.raw_timestamp) / self.n_gpsr as u32;
        let valid = packets.iter().fold(true, |acc, gps| {
            acc | ((self.timestamp as i32 - gps.raw_timestamp as i32).abs() < 10)
        });
        self.update_hw_test(valid, 3); // Valid if each timestamp within 1 sec of average

        // Update Data
        self.ingest(packets, param_bus);

        // Check Pos Difference
        let rcheck = true; // TODO: Update to compre pos w/ prev est bus
        self.update_hw_test(rcheck, 4);
    }

    fn ingest(&mut self, packets: &[Self::Packet], _param_bus: &ParamBus) {
        // Move to Self
        self.r_eci
            .assign(&Vector3::from_shape_fn((3, self.n_gpsr), |(row, col)| {
                packets[row].raw_r_eci[col]
            }));
        self.v_eci
            .assign(&Vector3::from_shape_fn((3, 1), |(row, col)| {
                packets[row].raw_v_eci[col]
            }));
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
        04: GPS reported POS < 1km of prev est
        03: All GPS Timestamp < 1 sec from average
        02: >n/2 GPS Valid
        01: MsgCounter Increasing
        00: Enabled
        LSB */
        self.error_code
    }
}

impl SensProcGPSRBus {
    pub fn initialize(n_gpsr: usize) -> Self {
        Self {
            timestamp: 0,
            n_gpsr,
            error_code: 0u16,
            prev_msg_counter: 0u32,
            r_eci: Vector3::zeros((3, n_gpsr)),
            v_eci: Vector3::zeros((3, n_gpsr)),
        }
    }

    fn update_hw_test(&mut self, flag: bool, bit_id: u8) {
        if bit_id > 15 {
            panic!("Invalid bit setting for u16 bitpack")
        }
        self.error_code ^= (0b1 * !flag as u16) << bit_id;
    }
}

impl Default for SensProcGPSRBus {
    fn default() -> Self {
        let n_gpsr = 1;
        Self::initialize(n_gpsr)
    }
}
