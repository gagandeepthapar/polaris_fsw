use altai_rs::meta::types::Generic1D;

use crate::estimation::types::EstimationBus;

#[derive(Clone, Debug, Default)]
pub struct ReferenceBus {}

impl ReferenceBus {
    pub fn process(_curr_est: &EstimationBus, _prev_ref: &ReferenceBus) -> Self {
        Self {}
    }
}

pub enum Reference {
    IDLE, // N/A
    SLEW, // MID-SLEW
    IPT,  // Inertial Point Track
}

pub trait ValidReference {
    fn q_ref_eci(&self) -> (Generic1D, bool);
    fn omega_ref(&self) -> (Generic1D, bool);
    fn alpha_ref(&self) -> (Generic1D, bool);
}

pub fn get_reference<T: ValidReference>(ref_cmd: T) -> (Generic1D, Generic1D, Generic1D, bool) {
    let (q_ref, q_err) = ref_cmd.q_ref_eci();
    let (o_ref, o_err) = ref_cmd.omega_ref();
    let (a_ref, a_err) = ref_cmd.alpha_ref();
    (q_ref, o_ref, a_ref, q_err | o_err | a_err)
}
