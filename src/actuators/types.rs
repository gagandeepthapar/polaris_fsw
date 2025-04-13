use crate::control::types::ControlBus;

#[derive(Clone)]
pub struct ActuatorBus {}

impl ActuatorBus {
    pub fn process(curr_ctrl: &ControlBus) -> Self {
        Self {}
    }
}
