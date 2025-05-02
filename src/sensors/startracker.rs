use crate::estimation::types::EstimationBus;
use crate::{fsw_types::ParamBus, sensors::types::*};
use altai_rs::{quatlib::qxform, types::*};
use ndarray::{concatenate, Axis};

#[derive(Debug, Default, Clone, Copy)]
pub struct RawStarTrackerPacket {
    // Timestamped attitude coming direct from startracker
    // Meta
    raw_timestamp: u32,
    raw_valid: bool,
    msg_counter: u32,

    // Sensor-Specific
    raw_q_sta_eci: [f64; 4],
    // TODO: ADD COV
}
impl RawSensorPacket for RawStarTrackerPacket {}

impl RawStarTrackerPacket {
    pub fn plant_update(
        &mut self,
        timestamp: u32,
        raw_valid: bool,
        inc_msg: bool,
        raw_q_sta_eci: [f64; 4],
    ) {
        self.raw_timestamp = timestamp;
        self.raw_valid = raw_valid;
        self.msg_counter += inc_msg as u32;
        self.raw_q_sta_eci = raw_q_sta_eci;
    }
}

#[derive(Debug, Clone)]
pub struct SensProcStarTrackerBus {
    // Processed data coming off STA
    // Meta
    timestamp: u32,
    error_code: u16,
    n_sta: usize,
    prev_msg_counter: u32,

    // Sensor Specific
    q_sc_eci: Quaternion4, // STA-reported attitude in SC Frame
                           // TODO: ADD COV
}

impl Sensor for SensProcStarTrackerBus {
    type Packet = RawStarTrackerPacket;
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
        let msg_inc = packets.iter().fold(true, |flag, sta| {
            flag & (sta.msg_counter != self.prev_msg_counter)
        });
        self.prev_msg_counter = packets[0].msg_counter;
        self.update_hw_test(msg_inc, 1); // HW Valid if MSG Counter Incrementing

        // Check Raw Valid
        let valid = packets
            .iter()
            .fold(0, |acc, sta| acc + sta.raw_valid as usize)
            > self.n_sta / 2;
        self.update_hw_test(valid, 2); // Valid if >half STA is valid

        // Check timestamp staleness
        self.timestamp =
            packets.iter().fold(0, |acc, sta| acc + sta.raw_timestamp) / self.n_sta as u32;
        let valid = packets.iter().fold(true, |acc, sta| {
            acc | ((self.timestamp as i32 - sta.raw_timestamp as i32).abs() < 10)
        });
        self.update_hw_test(valid, 3); // Valid if each timestamp within 1 sec of average

        // Update Data
        self.ingest(packets, param_bus);

        // Check reported quaternion
        let ang_check = true; // TODO: implement angular check against prev est bus
        self.update_hw_test(ang_check, 4);
    }

    fn ingest(&mut self, packets: &[Self::Packet], param_bus: &ParamBus) {
        // Transform to SC frame
        let tfr_sta = qxform(
            &param_bus.acs_sensors.q_sc_sta,
            &Quaternion4::from_shape_fn((4, self.n_sta), |(row, col)| {
                packets[col].raw_q_sta_eci[row]
            }),
        );

        // Move to Self
        self.q_sc_eci.assign(&tfr_sta);

        // Get timestamp average
        self.timestamp =
            packets.iter().fold(0, |acc, sta| acc + sta.raw_timestamp) / self.n_sta as u32;

        // Valid if >half of stas are valid
        let valid = packets
            .iter()
            .fold(0, |acc, sta| acc + sta.raw_valid as usize)
            > self.n_sta / 2;

        self.error_code = (!valid) as u16; // TODO: FIX
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
        04: Reported quaternion < 10 deg from previous reading
        03: All STA Timestamp < 1 sec from average
        02: >n/2 STA Valid
        01: MsgCounter Increasing
        00: Enabled
        LSB */
        self.error_code
    }
}

impl SensProcStarTrackerBus {
    pub fn initialize(n_sta: usize) -> Self {
        Self {
            timestamp: 0,
            error_code: 0u16,
            prev_msg_counter: 0u32,
            n_sta,
            q_sc_eci: concatenate![
                Axis(0),
                Generic2D::zeros((3, n_sta)),
                Generic2D::ones((1, n_sta))
            ],
        }
    }

    fn update_hw_test(&mut self, flag: bool, bit_id: u8) {
        if bit_id > 15 {
            panic!("Invalid bit setting for u16 bitpack")
        }
        self.error_code ^= (0b1 * !flag as u16) << bit_id;
    }
}

impl Default for SensProcStarTrackerBus {
    fn default() -> Self {
        let n_sta = 1;
        Self::initialize(n_sta)
    }
}
