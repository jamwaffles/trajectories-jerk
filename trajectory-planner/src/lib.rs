#[derive(Debug, Copy, Clone)]
pub struct TrajectorySegment {
    pub start: f32,
    pub end: f32,
    pub start_velocity: f32,
    pub end_velocity: f32,
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

impl TrajectorySegment {
    pub fn new(start: f32, end: f32, start_velocity: f32, end_velocity: f32) -> Self {
        Self {
            start,
            end,
            start_velocity,
            end_velocity,
        }
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

    fn compute_deltas(&self, max_acceleration: f32, max_velocity: f32) -> Deltas {
        let mut max_velocity = max_velocity;

        // Time to accelerate to Vmax
        let mut t1 = (max_velocity - self.start_velocity) / max_acceleration;

        // Time to decelerate to Vfinal
        // NOTE: I subtract Vfinal here where the paper assumes final velocity is zero
        let mut t3 = (max_velocity - self.end_velocity) / max_acceleration;

        // Distance from Vinitial to Vmax
        let mut x1 = Self::second_order(t1, 0.0, self.start_velocity, max_acceleration);

        // Distance from Vmax to Vfinal
        let mut x3 = Self::second_order(t3, 0.0, max_velocity, -max_acceleration);

        let mut t2 = (self.end - (self.start + x1 + x3)) / max_velocity;

        // Negative cruise duration - we need to shorten accel/decel to create a wedge shaped
        // profile
        if t2 < 0.0 {
            // Recalculate max velocity based on wedge profile calculation
            max_velocity = (max_acceleration * (self.end - self.start)
                + (0.5 * self.start_velocity.powi(2)))
            .sqrt();

            // Time to accelerate to Vmax
            t1 = (self.start_velocity - max_velocity) / max_acceleration;

            t2 = 0.0;

            // Distance from Vinitial to Vmax
            x1 = Self::second_order(t1, 0.0, self.start_velocity, max_acceleration);

            // Distance from Vmax to Vfinal
            x3 = Self::second_order(t3, 0.0, max_velocity, -max_acceleration);

            t3 = ((max_velocity - self.start_velocity) / max_acceleration)
                + t2
                + ((max_velocity - self.end_velocity) / -max_acceleration);
        }

        let x2 = max_velocity * t2;

        Deltas {
            t1,
            t2,
            t3,
            x1,
            x2,
            x3,
        }
    }

    /// Get total duration
    pub fn duration(&self, max_acceleration: f32, max_velocity: f32) -> f32 {
        let Deltas { t1, t2, t3, .. } = self.compute_deltas(max_acceleration, max_velocity);

        t1 + t2 + t3
    }

    /// Get the position at a given time
    pub fn position(&self, time: f32, max_acceleration: f32, max_velocity: f32) -> f32 {
        let deltas = self.compute_deltas(max_acceleration, max_velocity);

        if time <= deltas.t1 {
            // Acceleration phase
            Self::second_order(time, self.start, self.start_velocity, max_acceleration)
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            Self::second_order(time - deltas.t1, self.start + deltas.x1, max_velocity, 0.0)
        } else {
            // Deceleration phase
            Self::second_order(
                time - deltas.t1 - deltas.t2,
                self.start + deltas.x1 + deltas.x2,
                max_velocity,
                -max_acceleration,
            )
        }
    }

    /// Get the velocity (derivative of position) at a given time
    pub fn velocity(&self, time: f32, max_acceleration: f32, max_velocity: f32) -> f32 {
        let deltas = self.compute_deltas(max_acceleration, max_velocity);

        if time <= deltas.t1 {
            // Acceleration phase
            max_acceleration * time + self.start_velocity
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            max_velocity
        } else {
            // Deceleration phase
            max_velocity - max_acceleration * (time - deltas.t1 - deltas.t2)
        }
    }

    /// Get the acceleration at a given time
    pub fn acceleration(&self, time: f32, max_acceleration: f32, max_velocity: f32) -> f32 {
        let deltas = self.compute_deltas(max_acceleration, max_velocity);

        if time <= deltas.t1 {
            // Acceleration phase
            max_acceleration
        } else if time > deltas.t1 && time <= deltas.t1 + deltas.t2 {
            // Cruise phase
            0.0
        } else {
            // Deceleration phase
            -max_acceleration
        }
    }
}
