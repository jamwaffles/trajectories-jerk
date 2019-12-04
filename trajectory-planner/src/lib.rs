#[derive(Debug, Copy, Clone)]
pub struct TrajectorySegment {
    pub start: f32,
    pub end: f32,
    pub start_velocity: f32,
    pub end_velocity: f32,
    limits: Limits,
    deltas: Deltas,
}

#[derive(Debug, Copy, Clone)]
struct Deltas {
    pub t1: f32,
    pub t2: f32,
    pub t3: f32,
    pub x1: f32,
    pub x2: f32,
    pub x3: f32,
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
        let (deltas, limits) =
            Self::compute_deltas_and_limits(start, end, start_velocity, end_velocity, limits);

        Self {
            start,
            end,
            start_velocity,
            end_velocity,
            limits,
            deltas,
        }
    }

    // /// Set new velocity/acceleration limits and update internal values
    // pub fn set_limits(&mut self, limits: Limits) {
    //     let (deltas, limits) = Self::compute_deltas_and_limits(
    //         self.start,
    //         self.end,
    //         self.start_velocity,
    //         self.end_velocity,
    //         limits,
    //     );

    //     self.limits = limits;
    //     self.deltas = deltas;
    // }

    pub fn set_velocity_limit(&mut self, limit: f32) {
        self.limits.velocity = limit;

        let (deltas, limits) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        );

        self.limits = limits;
        self.deltas = deltas;
    }

    pub fn set_acceleration_limit(&mut self, limit: f32) {
        self.limits.acceleration = limit;

        let (deltas, limits) = Self::compute_deltas_and_limits(
            self.start,
            self.end,
            self.start_velocity,
            self.end_velocity,
            self.limits,
        );

        self.limits = limits;
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

    fn compute_deltas_and_limits(
        start: f32,
        end: f32,
        start_velocity: f32,
        end_velocity: f32,
        limits: Limits,
    ) -> (Deltas, Limits) {
        let mut limits = limits;

        // Time to accelerate to Vmax
        let mut t1 = (limits.velocity - start_velocity) / limits.acceleration;

        // Time to decelerate to Vfinal
        // NOTE: I subtract Vfinal here where the paper assumes final velocity is zero
        let mut t3 = (limits.velocity - end_velocity) / limits.acceleration;

        // Distance from Vinitial to Vmax
        let mut x1 = Self::second_order(t1, 0.0, start_velocity, limits.acceleration);

        // Distance from Vmax to Vfinal
        let mut x3 = Self::second_order(t3, 0.0, limits.velocity, -limits.acceleration);

        let mut t2 = (end - (start + x1 + x3)) / limits.velocity;

        // Negative cruise duration - we need to shorten accel/decel to create a wedge shaped
        // profile
        if t2 < 0.0 {
            // Recalculate max velocity based on wedge profile calculation
            limits.velocity =
                (limits.acceleration * (end - start) + (0.5 * start_velocity.powi(2))).sqrt();

            // Time to accelerate to Vmax
            t1 = (start_velocity - limits.velocity) / limits.acceleration;

            t2 = 0.0;

            // Distance from Vinitial to Vmax
            x1 = Self::second_order(t1, 0.0, start_velocity, limits.acceleration);

            // Distance from Vmax to Vfinal
            x3 = Self::second_order(t3, 0.0, limits.velocity, -limits.acceleration);

            t3 = ((limits.velocity - start_velocity) / limits.acceleration)
                + t2
                + ((limits.velocity - end_velocity) / -limits.acceleration);
        }

        let x2 = limits.velocity * t2;

        (
            Deltas {
                t1,
                t2,
                t3,
                x1,
                x2,
                x3,
            },
            limits,
        )
    }

    /// Get total duration
    pub fn duration(&self) -> f32 {
        let Deltas { t1, t2, t3, .. } = self.deltas;

        t1 + t2 + t3
    }

    /// Get the position at a given time
    pub fn position(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time <= deltas.t1 {
            // Acceleration phase
            Self::second_order(
                time,
                self.start,
                self.start_velocity,
                self.limits.acceleration,
            )
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            Self::second_order(
                time - deltas.t1,
                self.start + deltas.x1,
                self.limits.velocity,
                0.0,
            )
        } else {
            // Deceleration phase
            Self::second_order(
                time - deltas.t1 - deltas.t2,
                self.start + deltas.x1 + deltas.x2,
                self.limits.velocity,
                -self.limits.acceleration,
            )
        }
    }

    /// Get the velocity (derivative of position) at a given time
    pub fn velocity(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time <= deltas.t1 {
            // Acceleration phase
            self.limits.acceleration * time + self.start_velocity
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            self.limits.velocity
        } else {
            // Deceleration phase
            self.limits.velocity - self.limits.acceleration * (time - deltas.t1 - deltas.t2)
        }
    }

    /// Get the acceleration at a given time
    pub fn acceleration(&self, time: f32) -> f32 {
        let Self { deltas, .. } = self;

        if time <= deltas.t1 {
            // Acceleration phase
            self.limits.acceleration
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            0.0
        } else {
            // Deceleration phase
            -self.limits.acceleration
        }
    }
}
