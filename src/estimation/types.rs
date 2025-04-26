use crate::sensors::types::SensorBus;

#[derive(Clone, Debug, Default)]
pub struct EstimationBus {}

impl EstimationBus {
    pub fn process(_tlm_sensor: &SensorBus, _prev_est: &EstimationBus) -> Self {
        Self {}
    }
}
