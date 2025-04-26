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

pub trait Param {}

#[derive(Clone, Default, Debug)]
pub struct ParamBus {
    acs_actuators: ActuatorArchitecture,
    acs_control: ControlArchitecture,
    acs_estimation: EstimationArchitecture,
    acs_reference: ReferenceArchitecture,
    acs_sensors: SensorArchitecture,
    acs_multibody: MultibodyArchitecture,
}

#[derive(Clone, Default, Debug)]
pub struct ActuatorArchitecture {}
impl Param for ActuatorArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct ControlArchitecture {}
impl Param for ControlArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct EstimationArchitecture {}
impl Param for EstimationBus {}

#[derive(Clone, Default, Debug)]
pub struct ReferenceArchitecture {}
impl Param for ReferenceArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct SensorArchitecture {}
impl Param for SensorArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct MultibodyArchitecture {
    j_multibody: Vector3,
}
impl Param for MultibodyArchitecture {}
