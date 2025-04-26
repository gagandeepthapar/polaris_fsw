use crate::{estimation::types::EstimationBus, reference::types::ReferenceBus};

#[derive(Clone, Debug, Default)]
pub struct ControlBus {}

impl ControlBus {
    pub fn process(_curr_est: &EstimationBus, _curr_ref: &ReferenceBus) -> Self {
        Self {}
    }
}
