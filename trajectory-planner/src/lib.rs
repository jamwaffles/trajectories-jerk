#[macro_use]
extern crate log;

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
            Self::compute_deltas_and_limits(start, end, start_velocity, end_velocity, limits);

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
        );

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
        );

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
        );

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
        );

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
        initial_position + (initial_velocity * time) + ((0.5 * acceleration) * time.powi(2))
    }

    /// Compute the distance taken to go from initial velocity to a full stop
    ///
    /// This is denoted as `Xstop` in the paper
    fn distance_to_full_stop(start: f32, start_velocity: f32, limits: &Limits) -> f32 {
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
    ) -> (Deltas, f32) {
        // Compute distance to full stop (`Xstop`) from start (`X0`)
        let distance_to_full_stop = Self::distance_to_full_stop(start, start_velocity, &limits);

        // Get the sign of acceleration (`d`)
        let accel_sign = if start_velocity != 0.0 {
            (end - distance_to_full_stop).signum()
        } else {
            (end - start).signum()
        };

        let accel = limits.acceleration * accel_sign;
        let decel = limits.acceleration * -accel_sign;
        let mut max_reachable_velocity = limits.velocity * accel_sign;

        // Time to accelerate to Vmax. Note the call to `.abs()` to deal with cases `start_velocity`
        // is greater the achievable limit.
        let mut dt1 = (max_reachable_velocity - start_velocity).abs() / accel;

        // Time to decelerate to Vfinal
        // NOTE: I subtract Vfinal here where the paper assumes final velocity is zero
        let mut dt3 = ((max_reachable_velocity - end_velocity) * -accel_sign) / decel;

        // Distance from Vinitial to Vmax
        let mut dx1 = Self::second_order(dt1, 0.0, start_velocity, accel);

        // Distance from Vmax to Vfinal
        let mut dx3 = Self::second_order(dt3, 0.0, max_reachable_velocity, decel);

        let mut dt2 = (end - (start + dx1 + dx3)) / max_reachable_velocity;

        // Negative cruise duration - we need to shorten accel/decel to create a wedge shaped
        // profile
        if dt2 <= 0.0 {
            // Recalculate max velocity based on wedge profile max velocity
            max_reachable_velocity =
                (accel * (end - start) + (0.5 * start_velocity.powi(2))).sqrt();

            // Time to accelerate to Vmax
            dt1 = (max_reachable_velocity * accel_sign - start_velocity) / accel;

            dt2 = 0.0;

            dt3 = (max_reachable_velocity * accel_sign - end_velocity) / accel;

            // Distance from Vinitial to Vmax
            dx1 = Self::second_order(dt1, 0.0, start_velocity, accel);

            // Distance from Vmax to Vfinal
            dx3 = Self::second_order(dt3, 0.0, max_reachable_velocity, decel);
        }

        let dx2 = max_reachable_velocity * dt2;

        (
            Deltas {
                dt1,
                dt2,
                dt3,
                dx1,
                dx2,
                dx3,
            },
            max_reachable_velocity,
        )
    }

    /// Get total duration
    pub fn duration(&self) -> f32 {
        let Deltas { dt1, dt2, dt3, .. } = self.deltas;

        dt1 + dt2 + dt3
    }

    /// Get the position at a given time
    pub fn position(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time <= deltas.dt1 {
            // Acceleration phase
            Self::second_order(
                time,
                self.start,
                self.start_velocity,
                self.limits.acceleration,
            )
        } else if time > deltas.dt1 && time <= deltas.dt1 + deltas.dt2 {
            // Cruise phase
            Self::second_order(
                time - deltas.dt1,
                self.start + deltas.dx1,
                self.limits.velocity,
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

        if time <= deltas.dt1 {
            // Acceleration phase
            self.limits.acceleration * time + self.start_velocity
        } else if time > deltas.dt1 && time <= deltas.dt1 + deltas.dt2 {
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

        if time <= deltas.dt1 {
            // Acceleration phase
            self.limits.acceleration
        } else if time > deltas.dt1 && time <= deltas.dt1 + deltas.dt2 {
            // Cruise phase
            0.0
        } else {
            // Deceleration phase
            -self.limits.acceleration
        }
    }
}
