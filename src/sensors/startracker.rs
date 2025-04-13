use std::f64::consts::PI;
use std::u16::MAX;

use altai_rs::{
    meta::types::{Generic1D, Quaternion4},
    quatlib::qangle,
};
use ndarray::{array, concatenate, Axis};

use crate::sensors::types::*;

#[derive(Clone)]
pub struct RawStarTrackerPacket {
    id: u32,
    msg_counter: u32,
    valid_data: bool,
    timestamp: u32,
    q_sc_eci: Quaternion4,
    num_stars: u16,
}

pub struct StarTrackerBus {
    timestamp: u32,
    q_sc_eci: Generic1D,
    valid: bool,
}

pub struct StarTracker {
    prev_packet: RawStarTrackerPacket,
    curr_packet: RawStarTrackerPacket,
}

impl Default for RawStarTrackerPacket {
    fn default() -> Self {
        Self {
            id: 0,
            msg_counter: 0,
            valid_data: true,
            timestamp: 0,
            q_sc_eci: array![0., 0., 0., 1.]
                .into_shape_with_order((4, 1))
                .unwrap(),
            num_stars: MAX,
        }
    }
}

impl RawSensorPacket for RawStarTrackerPacket {
    fn hw_valid(&self) -> bool {
        self.valid_data
    }
    fn sensor_id(&self) -> u32 {
        self.id
    }
    fn timestamp(&self) -> u32 {
        self.timestamp
    }
    fn msg_counter(&self) -> u32 {
        self.msg_counter
    }
    fn data(&self) -> Generic1D {
        concatenate![
            Axis(0),
            self.q_sc_eci.clone().remove_axis(Axis(1)),
            array![self.num_stars as f64]
        ]
    }
}

impl Default for StarTracker {
    fn default() -> Self {
        Self {
            prev_packet: RawStarTrackerPacket::default(),
            curr_packet: RawStarTrackerPacket::default(),
        }
    }
}

impl Sensor for StarTracker {
    type Packet = RawStarTrackerPacket;

    fn read(&mut self, packet: &Self::Packet) {
        self.prev_packet = std::mem::replace(&mut self.curr_packet, packet.clone());
    }

    fn hardware_subtest(&self) -> u16 {
        let mut bools = [true; 16];
        let mut ii = 0;
        // BIT 0: INCREMENTING MESSAGE
        bools[ii] = true;
        ii += 1;

        // BIT 1: VALID SIGNAL
        bools[ii] = true;
        ii += 1;

        // BIT 2: QDIFF < 5 DEG/TS
        let qdiff = qangle(&self.prev_packet.q_sc_eci, &self.curr_packet.q_sc_eci);
        bools[ii] = qdiff[0] <= 5. * PI / 180.;

        bools
            .iter()
            .take(16)
            .enumerate()
            .fold(0u16, |acc, (i, &b)| acc | ((b as u16) << i))
    }

    fn process(&mut self) -> (Generic1D, bool, u16) {
        let hw_test = self.hardware_subtest();
        (
            self.curr_packet.q_sc_eci.clone().remove_axis(Axis(1)),
            hw_test == 0,
            hw_test,
        )
    }
}
