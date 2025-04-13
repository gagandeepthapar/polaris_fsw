use std::f64::consts::PI;
use std::u16::MAX;

use altai_rs::{
    meta::types::{Generic1D, Quaternion4},
    quatlib::qangle,
};
use ndarray::{array, concatenate, Axis};

use crate::sensors::types::*;

#[derive(Clone)]
pub struct RawIMUPacket {
    id: u32,
    msg_counter: u32,
    valid_data: bool,
    timestamp: u32,
    q_sc_eci: Quaternion4,
    num_stars: u16,
}

pub struct IMUBus {
    timestamp: u32,
    q_sc_eci: Generic1D,
    valid: bool,
}

pub struct IMU {
    prev_packet: RawIMUPacket,
    curr_packet: RawIMUPacket,
}

impl RawSensorPacket for RawIMUPacket {
    fn data(&self) -> Generic1D {
        Generic1D::zeros(1)
    }
    fn hw_valid(&self) -> bool {
        true
    }
    fn sensor_id(&self) -> u32 {
        0u32
    }
    fn msg_counter(&self) -> u32 {
        0u32
    }
    fn timestamp(&self) -> u32 {
        0u32
    }
}

impl Sensor for IMU {
    type Packet = RawIMUPacket;

    fn read(&mut self, packet: &Self::Packet) {}
    fn process(&mut self) -> (Generic1D, bool, u16) {
        (Generic1D::zeros(1), true, 0u16)
    }
    fn hardware_subtest(&self) -> u16 {
        0u16
    }
}
