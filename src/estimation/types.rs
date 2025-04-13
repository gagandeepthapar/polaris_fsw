use crate::sensors::types::SensorBus;

#[derive(Clone)]
pub struct EstimationBus {}

impl EstimationBus {
    pub fn process(tlm_sensor: &SensorBus, prev_est: &EstimationBus) -> Self {
        Self {}
    }
}
