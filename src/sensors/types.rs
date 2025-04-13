use crate::{estimation::types::EstimationBus, sensors::startracker};
use altai_rs::meta::types::Generic1D;

#[derive(Clone)]
pub struct RawSensorBus {}

impl Default for RawSensorBus {
    fn default() -> Self {
        Self {}
    }
}

// impl Bus for RawSensorBus {
//     type Prior;
//     fn process(data: Option<&Self::Prior>) -> Self {
//         Self {}
//     }
// }

pub trait RawSensorPacket {
    fn sensor_id(&self) -> u32;
    fn timestamp(&self) -> u32;
    fn msg_counter(&self) -> u32;
    fn hw_valid(&self) -> bool;
    fn data(&self) -> Generic1D;
}

pub trait Sensor {
    type Packet: RawSensorPacket;

    fn read(&mut self, packet: &Self::Packet);
    fn process(&mut self) -> (Generic1D, bool, u16);
    fn hardware_subtest(&self) -> u16;
}

#[derive(Clone)]
pub struct SensorBus {}

impl SensorBus {
    pub fn process(raw_sensor_data: &RawSensorBus, prev_est: &EstimationBus) -> Self {
        Self {}
    }
}
