use polaris_log::types::PolarisLogger;

pub mod actuators;
pub mod control;
pub mod estimation;
pub mod fsw_types;
pub mod reference;
pub mod sensors;

use actuators::types::ActuatorBus;
use control::types::ControlBus;
use estimation::types::EstimationBus;
use fsw_types::GNCState;
use reference::types::ReferenceBus;
use sensors::types::{RawSensorBus, SensorBus};

pub struct FlightSoftware {
    _logger: PolarisLogger,
    prev_state: GNCState,
    curr_state: GNCState,
}

impl FlightSoftware {
    // Initialize FSW / Consts
    pub fn initialize(logger: PolarisLogger) -> Self {
        Self {
            _logger: logger,
            prev_state: GNCState::default(),
            curr_state: GNCState::default(),
        }
    }

    // "GNC Loop" -> outputs Actuator Commands
    pub fn gnc_loop(&mut self, raw_sensor_bus: &mut RawSensorBus) -> ActuatorBus {
        // overwrite previous
        std::mem::swap(&mut self.curr_state, &mut self.prev_state);

        // read sensors
        self.curr_state.raw_sensor_bus = std::mem::take(raw_sensor_bus);
        self.curr_state.tlm_sensor_bus = SensorBus::process(
            &self.curr_state.raw_sensor_bus,
            &self.prev_state.estimation_bus,
        );

        // estimate state
        self.curr_state.estimation_bus = EstimationBus::process(
            &self.curr_state.tlm_sensor_bus,
            &self.prev_state.estimation_bus,
        );

        // compute reference
        self.curr_state.reference_bus = ReferenceBus::process(
            &self.curr_state.estimation_bus,
            &self.prev_state.reference_bus,
        );

        // compute control error and commands
        self.curr_state.control_bus = ControlBus::process(
            &self.curr_state.estimation_bus,
            &self.curr_state.reference_bus,
        );

        // compute actuator commands
        self.curr_state.actuator_bus = ActuatorBus::process(&self.curr_state.control_bus);

        self.curr_state.actuator_bus.clone()
    }

    // fn read_sensors(&mut self, raw_sensor_bus: Generic1D) {}
}
