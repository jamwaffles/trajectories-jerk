#[macro_use]
extern crate log;

use std::error::Error;

#[derive(Debug, Copy, Clone)]

pub struct TrajectorySegment {
    start: f32,
    end: f32,
    start_velocity: f32,
    end_velocity: f32,
    limits: Limits,
    deltas: Deltas,
    max_reachable_velocity: f32,
}

#[derive(Debug, Copy, Clone)]
struct Deltas {
    pub dt1: f32,
    pub dt2: f32,
    pub dt3: f32,
    pub dx1: f32,
    pub dx2: f32,
    pub dx3: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct Limits {
    pub acceleration: f32,
    pub velocity: f32,
}

impl TrajectorySegment {
    pub fn new(
        start: f32,
        end: f32,
        start_velocity: f32,
        end_velocity: f32,
        limits: Limits,
    ) -> Self {
        let (deltas, max_reachable_velocity) =
            Self::compute_deltas_and_limits(start, end, start_velocity, end_velocity, limits)
                .unwrap();

        Self {
            start,
            end,
            start_velocity,
            end_velocity,
            limits,
            deltas,
            max_reachable_velocity,
        }
    }
    pub fn set_velocity_limit(&mut self, limit: f32) {
        self.limits.velocity = limit;

        let (deltas, max_reachable_velocity) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        )
        .unwrap();

        self.max_reachable_velocity = max_reachable_velocity;
        self.deltas = deltas;
    }

    pub fn set_acceleration_limit(&mut self, limit: f32) {
        self.limits.acceleration = limit;

        let (deltas, max_reachable_velocity) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        )
        .unwrap();

        self.max_reachable_velocity = max_reachable_velocity;
        self.deltas = deltas;
    }

    pub fn set_start_velocity(&mut self, velocity: f32) {
        self.start_velocity = velocity;

        let (deltas, max_reachable_velocity) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        )
        .unwrap();

        self.max_reachable_velocity = max_reachable_velocity;
        self.deltas = deltas;
    }

    pub fn set_end_velocity(&mut self, velocity: f32) {
        self.end_velocity = velocity;

        let (deltas, max_reachable_velocity) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        )
        .unwrap();

        self.max_reachable_velocity = max_reachable_velocity;
        self.deltas = deltas;
    }

    /// Get position for a given time with initial parameters and constant acceleration
    ///
    /// TODO: Set up benchmarks and see if FMA (`.mul_add()`) speeds this up by much
    fn second_order(
        time: f32,
        initial_position: f32,
        initial_velocity: f32,
        acceleration: f32,
    ) -> f32 {
        initial_position + (initial_velocity * time) + (0.5 * acceleration * time.powi(2))
    }

    /// Compute the distance taken to go from initial velocity to a full stop
    ///
    /// This is denoted as `Xstop` in the paper
    fn position_at_full_stop(start: f32, start_velocity: f32, limits: &Limits) -> f32 {
        // Time to decelerate to 0 velocity
        let time = start_velocity / limits.acceleration;

        let distance = Self::second_order(time, start, start_velocity, -limits.acceleration);

        distance
    }

    fn compute_deltas_and_limits(
        start: f32,
        end: f32,
        start_velocity: f32,
        end_velocity: f32,
        limits: Limits,
    ) -> Result<(Deltas, f32), Box<dyn Error>> {
        let stop_pos = Self::position_at_full_stop(start, start_velocity, &limits);

        // Clamp final velocity at limit
        let end_velocity = end_velocity.min(limits.velocity);

        debug!("stop_pos {}", stop_pos);

        // Sign of acceleration
        let d = (end - stop_pos).signum();

        let mut d_accel = d;
        let d_decel = -d;

        if start_velocity > limits.velocity {
            d_accel = -d;
        }

        debug!("d {}, d_accel {}, d_decel {}", d, d_accel, d_decel);

        // `v`
        let mut max_reachable_velocity = d * limits.velocity;
        // `a_acc`
        let accel = limits.acceleration * d_accel;
        // `a_dec`
        let decel = limits.acceleration * d_decel;

        let mut dt1 = (max_reachable_velocity - start_velocity) / accel;
        let mut dt3 = ((max_reachable_velocity - end_velocity) / decel).abs();

        let mut dx1 = Self::second_order(dt1, 0.0, start_velocity, accel);
        let mut dx3 = Self::second_order(dt3, 0.0, max_reachable_velocity, decel);

        let mut dt2 = (end - (start + dx1 + dx3)) / max_reachable_velocity;

        if dt2 < 0.0 {
            debug!("LT0");
            max_reachable_velocity =
                (d_accel * (end - start) + (0.5 * (start_velocity - end_velocity).powi(2))).sqrt();

            debug!("New max reachable {}", max_reachable_velocity);

            dt1 = (d_accel * (max_reachable_velocity - start_velocity)) / accel;
            dt2 = 0.0;
            dt3 = (d_decel * (max_reachable_velocity - end_velocity)) / decel;

            dx1 = Self::second_order(dt1, 0.0, start_velocity, accel);
            dx3 = Self::second_order(dt3, 0.0, max_reachable_velocity, decel);
        }

        let dx2 = Self::second_order(dt2, 0.0, max_reachable_velocity, 0.0);

        let deltas = Deltas {
            dt1,
            dt2,
            dt3,
            dx1,
            dx2,
            dx3,
        };

        Self::validate_deltas(&deltas, start, end)?;

        Ok((deltas, max_reachable_velocity))
    }

    fn validate_deltas(deltas: &Deltas, start: f32, end: f32) -> Result<(), String> {
        let Deltas {
            dt1,
            dt2,
            dt3,
            dx1,
            dx2,
            dx3,
        } = deltas;

        let total_delta = end - start;

        // Times must be positive
        // TODO: Bench this vs infinite check and a gte 0
        if !dt1.is_finite() || !dt1.is_sign_positive() {
            return Err(format!("initial accel or decel time {} is negative", dt1));
        }
        if !dt2.is_finite() || !dt2.is_sign_positive() {
            return Err(format!("cruise time {} is negative", dt2));
        }
        if !dt3.is_finite() || !dt3.is_sign_positive() {
            return Err(format!("final decel time {} is negative", dt3));
        }

        let calculated_delta = dx1 + dx2 + dx3;

        // Sum of calculated deltas must equal total delta between waypoints
        if !approx::ulps_eq!(calculated_delta, total_delta) {
            return Err(format!(
                "calculated distance {:.32} does not equal total delta {:.32}",
                calculated_delta, total_delta
            ));
        }

        Ok(())
    }

    /// Get total duration
    pub fn duration(&self) -> f32 {
        let Deltas { dt1, dt2, dt3, .. } = self.deltas;

        dt1 + dt2 + dt3
    }

    /// Get the position at a given time
    pub fn position(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time < deltas.dt1 {
            // Acceleration phase
            Self::second_order(
                time,
                self.start,
                self.start_velocity,
                if self.start_velocity > self.max_reachable_velocity {
                    -self.limits.acceleration
                } else {
                    self.limits.acceleration
                },
            )
        } else if deltas.dt1 <= time && time < deltas.dt1 + deltas.dt2 {
            // Cruise phase
            Self::second_order(
                time - deltas.dt1,
                self.start + deltas.dx1,
                self.max_reachable_velocity,
                0.0,
            )
        } else {
            // Deceleration phase
            Self::second_order(
                time - deltas.dt1 - deltas.dt2,
                self.start + deltas.dx1 + deltas.dx2,
                self.max_reachable_velocity,
                -self.limits.acceleration,
            )
        }
    }

    /// Get the velocity (derivative of position) at a given time
    pub fn velocity(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time < deltas.dt1 {
            let accel = if self.start_velocity > self.max_reachable_velocity {
                -self.limits.acceleration
            } else {
                self.limits.acceleration
            };

            // Acceleration phase
            accel * time + self.start_velocity
        } else if deltas.dt1 <= time && time < deltas.dt1 + deltas.dt2 {
            // Cruise phase
            self.max_reachable_velocity
        } else {
            // Deceleration phase
            self.max_reachable_velocity
                - self.limits.acceleration * (time - deltas.dt1 - deltas.dt2)
        }
    }

    /// Get the acceleration at a given time
    pub fn acceleration(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time < deltas.dt1 {
            // Acceleration phase
            if self.start_velocity > self.max_reachable_velocity {
                -self.limits.acceleration
            } else {
                self.limits.acceleration
            }
        } else if deltas.dt1 <= time && time < deltas.dt1 + deltas.dt2 {
            // Cruise phase
            0.0
        } else {
            // Deceleration phase
            -self.limits.acceleration
        }
    }
}
