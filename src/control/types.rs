use crate::{estimation::types::EstimationBus, reference::types::ReferenceBus};

#[derive(Clone)]
pub struct ControlBus {}

impl ControlBus {
    pub fn process(curr_est: &EstimationBus, curr_ref: &ReferenceBus) -> Self {
        Self {}
    }
}
