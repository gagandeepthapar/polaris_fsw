pub mod actuators;
pub mod control;
pub mod estimation;
pub mod fsw_types;
pub mod reference;
pub mod sensors;

use actuators::types::ActuatorBus;
use fsw_types::{GNCState, ParamBus};
use sensors::types::RawSensorBus;

use log;

#[derive(Debug)]
pub struct FlightSoftware {
    param_bus: ParamBus,
    prev_state: GNCState,
    pub curr_state: GNCState,
}

impl FlightSoftware {
    // Initialize FSW / Consts
    pub fn initialize(fsw_params: ParamBus) -> Self {
        log::trace!("Initializing FSW");
        Self {
            param_bus: fsw_params,
            prev_state: GNCState::default(),
            curr_state: GNCState::default(),
        }
    }

    // "GNC Loop" -> outputs Actuator Commands
    pub fn gnc_loop(&mut self, raw_sensor_bus: &mut RawSensorBus) -> ActuatorBus {
        log::trace!("Running GNC FSW Loop");

        // overwrite previous
        std::mem::swap(&mut self.curr_state, &mut self.prev_state);

        // read sensors
        self.curr_state.raw_sensor_bus = std::mem::take(raw_sensor_bus);
        log::error!("{:?}", self.curr_state.raw_sensor_bus);

        self.curr_state.tlm_sensor_bus.process(
            // Current State
            &self.curr_state.raw_sensor_bus,
            // Previous State
            &self.prev_state.estimation_bus,
            // Params
            &self.param_bus,
        );
        log::error!("{:?}", self.curr_state.tlm_sensor_bus);

        // estimate state
        self.curr_state.estimation_bus.process(
            // Current State
            &self.curr_state.tlm_sensor_bus,
            // Previous State
            &self.prev_state.estimation_bus,
        );

        // compute reference
        self.curr_state.reference_bus.process(
            // Current State
            &self.curr_state.estimation_bus,
            // Previous State
            &self.prev_state.reference_bus,
        );

        // compute control error and commands
        self.curr_state.control_bus.process(
            // Current State
            &self.curr_state.estimation_bus,
            &self.curr_state.reference_bus,
            // Previous State
        );

        // compute actuator commands
        self.curr_state.actuator_bus.process(
            // Current State
            &self.curr_state.control_bus, // Previous State
        );

        self.curr_state.actuator_bus.clone()
    }

    // fn read_sensors(&mut self, raw_sensor_bus: Generic1D) {}
}
