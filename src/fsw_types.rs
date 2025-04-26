use altai_rs::meta::types::Vector3;

use crate::{
    actuators::types::ActuatorBus,
    control::types::ControlBus,
    estimation::types::EstimationBus,
    reference::types::ReferenceBus,
    sensors::types::{RawSensorBus, SensorBus},
};

#[derive(Clone, Debug, Default)]
pub struct GNCState {
    pub raw_sensor_bus: RawSensorBus,
    pub tlm_sensor_bus: SensorBus,
    pub estimation_bus: EstimationBus,
    pub reference_bus: ReferenceBus,
    pub control_bus: ControlBus,
    pub actuator_bus: ActuatorBus,
}
pub trait Param {}

#[derive(Clone, Default, Debug)]
pub struct ParamBus {
    pub acs_sensors: SensorArchitecture,
    pub acs_estimation: EstimationArchitecture,
    pub acs_reference: ReferenceArchitecture,
    pub acs_control: ControlArchitecture,
    pub acs_actuators: ActuatorArchitecture,

    pub acs_multibody: MultibodyArchitecture,
}
impl ParamBus {
    pub fn initialize(
        acs_sensors: SensorArchitecture,
        acs_estimation: EstimationArchitecture,
        acs_reference: ReferenceArchitecture,
        acs_control: ControlArchitecture,
        acs_actuators: ActuatorArchitecture,
        acs_multibody: MultibodyArchitecture,
    ) -> Self {
        Self {
            acs_sensors,
            acs_estimation,
            acs_reference,
            acs_control,
            acs_actuators,
            acs_multibody,
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct ActuatorArchitecture {}
impl Param for ActuatorArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct ControlArchitecture {}
impl Param for ControlArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct EstimationArchitecture {}
impl Param for EstimationArchitecture {}

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
impl MultibodyArchitecture {
    pub fn initialize(j_multibody: Vector3) -> Self {
        Self { j_multibody }
    }
}
impl Param for MultibodyArchitecture {}
