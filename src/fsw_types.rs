use crate::{
    actuators::types::ActuatorBus,
    control::types::ControlBus,
    estimation::types::EstimationBus,
    reference::types::ReferenceBus,
    sensors::types::{RawSensorBus, SensorBus},
};

#[derive(Clone)]
pub struct GNCState {
    pub raw_sensor_bus: RawSensorBus,
    pub tlm_sensor_bus: SensorBus,
    pub estimation_bus: EstimationBus,
    pub reference_bus: ReferenceBus,
    pub control_bus: ControlBus,
    pub actuator_bus: ActuatorBus,
}

impl Default for GNCState {
    fn default() -> Self {
        Self {
            raw_sensor_bus: RawSensorBus {},
            tlm_sensor_bus: SensorBus {},
            estimation_bus: EstimationBus {},
            reference_bus: ReferenceBus {},
            control_bus: ControlBus {},
            actuator_bus: ActuatorBus {},
        }
    }
}
