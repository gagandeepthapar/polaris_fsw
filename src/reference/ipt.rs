use crate::reference::types::ValidReference;
use altai_rs::{meta::types::Generic1D, quatlib::dcm2quat, veclib::mfcross};
use ndarray::{array, concatenate, s, Axis};

pub struct InertialPointTrack {
    // SV
    pointing_axis: Generic1D,
    power_axis: Generic1D,
    _rotation_axis: Generic1D,

    // ECI
    right_ascension: f64,
    declination: f64,
    roll: Option<f64>,
    u_sun: Option<Generic1D>,
}

impl InertialPointTrack {
    fn new(
        pointing_axis: Generic1D,
        power_axis: Generic1D,
        right_ascension: f64,
        declination: f64,
        roll: Option<f64>,
        u_sun: Option<Generic1D>,
    ) -> Self {
        Self {
            pointing_axis,
            power_axis,
            _rotation_axis: Generic1D::zeros(3),
            right_ascension,
            declination,
            roll,
            u_sun,
        }
    }
}

impl ValidReference for InertialPointTrack {
    fn alpha_ref(&self) -> (Generic1D, bool) {
        (Generic1D::zeros(3), false)
    }
    fn omega_ref(&self) -> (Generic1D, bool) {
        (Generic1D::zeros(3), false)
    }
    fn q_ref_eci(&self) -> (Generic1D, bool) {
        // default
        let mut q_ref_eci = array![0., 0., 0., 1.];

        let z_sv = &self.pointing_axis;
        let z_eci = array![
            self.right_ascension.cos() * self.declination.cos(),
            self.right_ascension.sin() * self.declination.cos(),
            self.declination.sin()
        ];

        if self.roll.is_some() {
            // Align Rotation to Roll
            let roll = self.roll.unwrap();
            let qvec = z_eci.to_owned() * (roll / 2.).sin();
            qvec.assign_to(q_ref_eci.slice_mut(s![0..3]));
            q_ref_eci[3] = (roll / 2.).cos();
        } else {
            // Align Power to Sun
            let x_sv = mfcross(&self.power_axis, &z_sv);
            let x_eci = mfcross(self.u_sun.as_ref().unwrap(), &z_eci);

            let t_sv = concatenate![
                Axis(1),
                x_sv.into_shape_with_order((3, 1)).unwrap(),
                self.power_axis
                    .to_owned()
                    .into_shape_with_order((3, 1))
                    .unwrap(),
                z_sv.to_owned().into_shape_with_order((3, 1)).unwrap()
            ];
            let t_eci = concatenate![
                Axis(1),
                x_eci.into_shape_with_order((3, 1)).unwrap(),
                self.u_sun
                    .as_ref()
                    .unwrap()
                    .to_owned()
                    .into_shape_with_order((3, 1))
                    .unwrap(),
                z_eci.to_owned().into_shape_with_order((3, 1)).unwrap()
            ];

            let t_sv_eci = (t_sv.t().dot(&t_eci)).t().to_owned();
            dcm2quat(t_sv_eci.insert_axis(Axis(2)))
                .remove_axis(Axis(1))
                .assign_to(q_ref_eci.slice_mut(s![0..4]));
        };

        (q_ref_eci, false)
    }
}
