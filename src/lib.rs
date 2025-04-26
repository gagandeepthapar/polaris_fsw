pub mod actuators;
pub mod control;
pub mod estimation;
pub mod fsw_types;
pub mod reference;
pub mod sensors;

use actuators::types::ActuatorBus;
use control::types::ControlBus;
use estimation::types::EstimationBus;
use fsw_types::{GNCState, ParamBus};
use reference::types::ReferenceBus;
use sensors::types::{RawSensorBus, SensorBus};

use log;

#[derive(Debug)]
pub struct FlightSoftware {
    // _logger: Option<PolarisLogger>,
    _param_bus: ParamBus,
    prev_state: GNCState,
    pub curr_state: GNCState,
}

impl FlightSoftware {
    // Initialize FSW / Consts
    pub fn initialize(fsw_params: ParamBus) -> Self {
        log::trace!("Initializing FSW");
        Self {
            // _logger: None,
            _param_bus: fsw_params,
            prev_state: GNCState::default(),
            curr_state: GNCState::default(),
        }
    }

    // "GNC Loop" -> outputs Actuator Commands
    pub fn gnc_loop(&mut self, raw_sensor_bus: &mut RawSensorBus) -> ActuatorBus {
        log::trace!("Running GNC FSW Loop");

        // overwrite previous
        std::mem::swap(&mut self.curr_state, &mut self.prev_state);
        log::trace!("{:?}", raw_sensor_bus);

        // read sensors
        self.curr_state.raw_sensor_bus = std::mem::take(raw_sensor_bus);
        self.curr_state.tlm_sensor_bus = SensorBus::process(
            // Current State
            &self.curr_state.raw_sensor_bus,
            // Previous State
            &self.prev_state.estimation_bus,
        );

        // estimate state
        self.curr_state.estimation_bus = EstimationBus::process(
            // Current State
            &self.curr_state.tlm_sensor_bus,
            // Previous State
            &self.prev_state.estimation_bus,
        );

        // compute reference
        self.curr_state.reference_bus = ReferenceBus::process(
            // Current State
            &self.curr_state.estimation_bus,
            // Previous State
            &self.prev_state.reference_bus,
        );

        // compute control error and commands
        self.curr_state.control_bus = ControlBus::process(
            // Current State
            &self.curr_state.estimation_bus,
            &self.curr_state.reference_bus,
            // Previous State
        );

        // compute actuator commands
        self.curr_state.actuator_bus = ActuatorBus::process(
            // Current State
            &self.curr_state.control_bus, // Previous State
        );

        self.curr_state.actuator_bus.clone()
    }

    // fn read_sensors(&mut self, raw_sensor_bus: Generic1D) {}
}
