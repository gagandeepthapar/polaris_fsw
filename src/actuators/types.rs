use crate::control::types::ControlBus;

#[derive(Clone, Debug, Default)]
pub struct ActuatorBus {}

impl ActuatorBus {
    pub fn process(_curr_ctrl: &ControlBus) -> Self {
        Self {}
    }
}
