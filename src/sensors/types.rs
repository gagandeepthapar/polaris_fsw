use super::gpsr::{RawGPSRPacket, SensProcGPSRBus};
use super::imu::SensProcIMUBus;
use super::startracker::{RawStarTrackerPacket, SensProcStarTrackerBus};
use crate::fsw_types::ParamBus;
use crate::sensors::imu::RawIMUPacket;

use crate::estimation::types::EstimationBus;

const MAX_IMU: usize = 12;
const MAX_STA: usize = 4;
const MAX_GPSR: usize = 1;

#[derive(Clone, Debug)]
pub struct RawSensorBus {
    raw_imu_bus: [RawIMUPacket; MAX_IMU],
    raw_sta_bus: [RawStarTrackerPacket; MAX_STA],
    raw_gpsr_bus: [RawGPSRPacket; MAX_GPSR],
}

impl Default for RawSensorBus {
    fn default() -> Self {
        Self {
            raw_imu_bus: [RawIMUPacket::default(); MAX_IMU],
            raw_sta_bus: [RawStarTrackerPacket::default(); MAX_STA],
            raw_gpsr_bus: [RawGPSRPacket::default(); MAX_GPSR],
        }
    }
}

pub trait RawSensorPacket {}

pub trait Sensor {
    type Packet: RawSensorPacket;
    fn ingest(&mut self, packets: &[Self::Packet], param_bus: &ParamBus); // Convert from sensor units to SI units; SC/ECI frame
    fn process(
        &mut self,
        packets: &[Self::Packet],
        prev_estimation_bus: &EstimationBus,
        param_bus: &ParamBus,
    ); // Check for validity, set hw subtest
    fn hardware_subtest(&self) -> u16;
}

#[derive(Clone, Debug, Default)]
pub struct SensorBus {
    imu_bus: SensProcIMUBus,
    imu_available: bool,
    sta_bus: SensProcStarTrackerBus,
    sta_available: bool,
    gpsr_bus: SensProcGPSRBus,
    gpsr_available: bool,
}

impl SensorBus {
    pub fn initialize(n_imu: usize, n_sta: usize, n_gpsr: usize) -> Self {
        // Check against max supported
        let n_imu = Self::check_max(n_imu, MAX_IMU, "IMUs");
        let n_sta = Self::check_max(n_sta, MAX_IMU, "STAs");
        let n_gpsr = Self::check_max(n_gpsr, MAX_IMU, "GPSRs");

        Self {
            imu_bus: SensProcIMUBus::initialize(n_imu),
            imu_available: n_imu > 0,
            sta_bus: SensProcStarTrackerBus::initialize(n_sta),
            sta_available: n_sta > 0,
            gpsr_bus: SensProcGPSRBus::initialize(n_gpsr),
            gpsr_available: n_gpsr > 0,
        }
    }

    pub fn process(
        &mut self,
        raw_sensor_data: &RawSensorBus,
        prev_est_bus: &EstimationBus,
        param_bus: &ParamBus,
    ) {
        // Update IMU
        if self.imu_available {
            self.imu_bus
                .process(&raw_sensor_data.raw_imu_bus, prev_est_bus, param_bus);
        }

        // Update STA
        if self.sta_available {
            self.sta_bus
                .process(&raw_sensor_data.raw_sta_bus, prev_est_bus, param_bus);
        }

        // Update GPSR
        if self.gpsr_available {
            self.gpsr_bus
                .process(&raw_sensor_data.raw_gpsr_bus, prev_est_bus, param_bus);
        }

        // TODO: Add CSS
        // TODO: Add SADA
        // TODO: Add RWA (Tach + TLM)
    }

    fn check_max(n_init: usize, max: usize, name: &str) -> usize {
        let n = {
            if n_init > max {
                log::error!(
                    "Too many {} instantiated; {} vs MAX: {}\nLimiting to MAX...",
                    name,
                    n_init,
                    max
                );
                max
            } else {
                n_init
            }
        };
        n
    }
}
