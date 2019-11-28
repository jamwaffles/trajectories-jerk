#[derive(Debug, Copy, Clone)]
pub struct TrajectorySegment {
    pub delta_t1: f32,
    pub delta_t2: f32,
    pub delta_t3: f32,
    pub delta_x1: f32,
    pub delta_x2: f32,
    pub delta_x3: f32,
    pub max_acceleration: f32,
    pub max_velocity: f32,
    pub start: f32,
    pub end: f32,
    pub initial_velocity: f32,
    pub final_velocity: f32,
    pub duration: f32,
}

impl TrajectorySegment {
    pub fn new(start: f32, end: f32, max_acceleration: f32, max_velocity: f32) -> Self {
        let mut max_velocity = max_velocity;

        let initial_velocity = 0.0;
        // TODO
        let final_velocity = 0.0;

        // Time to accelerate to Vmax
        let mut delta_t1 = (max_velocity - initial_velocity) / max_acceleration;

        // Time to decelerate to Vfinal
        // NOTE: I subtract Vfinal here where the paper assumes final velocity is zero
        let mut delta_t3 = (max_velocity - final_velocity) / max_acceleration;

        // Distance from Vinitial to Vmax
        let mut delta_x1 = Self::second_order(delta_t1, 0.0, initial_velocity, max_acceleration);

        // Distance from Vmax to Vfinal
        let mut delta_x3 = Self::second_order(delta_t3, 0.0, max_velocity, -max_acceleration);

        let mut delta_t2 = (end - (start + delta_x1 + delta_x3)) / max_velocity;

        // Negative cruise duration - we need to shorten accel/decel to create a wedge shaped
        // profile
        if delta_t2 < 0.0 {
            // Recalculate max velocity based on wedge profile calculation
            max_velocity =
                (max_acceleration * (end - start) + (0.5 * initial_velocity.powi(2))).sqrt();

            // Time to accelerate to Vmax
            delta_t1 = (initial_velocity - max_velocity) / max_acceleration;

            delta_t2 = 0.0;

            // Distance from Vinitial to Vmax
            delta_x1 = Self::second_order(delta_t1, 0.0, initial_velocity, max_acceleration);

            // Distance from Vmax to Vfinal
            delta_x3 = Self::second_order(delta_t3, 0.0, max_velocity, -max_acceleration);

            delta_t3 = ((max_velocity - initial_velocity) / max_acceleration)
                + delta_t2
                + ((max_velocity - final_velocity) / -max_acceleration);
        }

        let delta_x2 = max_velocity * delta_t2;

        Self {
            delta_t1,
            delta_t2,
            delta_t3,
            delta_x1,
            delta_x2,
            delta_x3,
            max_acceleration,
            max_velocity,
            start,
            end,
            initial_velocity,
            final_velocity,
            duration: delta_t1 + delta_t2 + delta_t3,
        }
    }

    /// Get the position at a given time
    pub fn position(&self, time: f32) -> f32 {
        if time <= self.delta_t1 {
            // Acceleration phase
            Self::second_order(
                time,
                self.start,
                self.initial_velocity,
                self.max_acceleration,
            )
        } else if time > self.delta_t1 && time <= self.delta_t1 + self.delta_t2 {
            // Cruise phase
            Self::second_order(
                time - self.delta_t1,
                self.start + self.delta_x1,
                self.max_velocity,
                0.0,
            )
        } else {
            // Deceleration phase
            Self::second_order(
                time - self.delta_t1 - self.delta_t2,
                self.start + self.delta_x1 + self.delta_x2,
                self.max_velocity,
                -self.max_acceleration,
            )
        }
    }

    /// Get the velocity (derivative of position) at a given time
    pub fn velocity(&self, time: f32) -> f32 {
        if time <= self.delta_t1 {
            // Acceleration phase
            self.max_acceleration * time + self.initial_velocity
        } else if time > self.delta_t1 && time <= self.delta_t1 + self.delta_t2 {
            // Cruise phase
            self.max_velocity
        } else {
            // Deceleration phase
            self.max_velocity - self.max_acceleration * (time - self.delta_t1 - self.delta_t2)
        }
    }

    /// Get the acceleration at a given time
    pub fn acceleration(&self, time: f32) -> f32 {
        if time <= self.delta_t1 {
            // Acceleration phase
            self.max_acceleration
        } else if time > self.delta_t1 && time <= self.delta_t1 + self.delta_t2 {
            // Cruise phase
            0.0
        } else {
            // Deceleration phase
            -self.max_acceleration
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
}
