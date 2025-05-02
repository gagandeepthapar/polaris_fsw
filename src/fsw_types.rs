use altai_rs::types::{Generic2D, Quaternion4};

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
pub struct SensorArchitecture {
    pub q_sc_imu: Quaternion4,
    pub q_sc_sta: Quaternion4,
}
impl Param for SensorArchitecture {}

#[derive(Clone, Default, Debug)]
pub struct MultibodyArchitecture {
    pub j_multibody: Generic2D,
}
impl MultibodyArchitecture {
    pub fn initialize(j_multibody: &Generic2D) -> Self {
        Self {
            j_multibody: j_multibody.to_owned(),
        }
    }
}
impl Param for MultibodyArchitecture {}
