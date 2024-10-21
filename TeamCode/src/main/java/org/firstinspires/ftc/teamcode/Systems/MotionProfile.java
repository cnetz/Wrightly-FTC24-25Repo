package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {

    // Fixed motion profile parameters as constants
    private static final double maxAcceleration = 2.0;  // Acceleration rate (power/second)
    private static final double maxVelocity = 1.0;      // Maximum velocity (motor power)
    private static final double Kp = 0.001;             // Proportional constant for position error
    private static final double Kv = 0.001;             // Constant for velocity feedback
    private static final double Ka = 0.0001;            // Constant for acceleration feedback
    private static final double positionTolerance = 10; // Tolerance for stopping motion (encoder ticks)

    // Internal timer to track elapsed time
    private final ElapsedTime timer;

    public MotionProfile() {
        timer = new ElapsedTime();
    }

    // Reset the timer to start the motion profile
    public void startMotionProfile() {
        timer.reset();  // Reset the internal timer
    }

    // Method to calculate motor power
    public double calculateMotorPower(double targetDistance, double currentPosition) {
        double elapsedTime = timer.seconds();  // Get the elapsed time
        double x = calculatePosition(targetDistance, elapsedTime);
        double v = calculateVelocity(targetDistance, elapsedTime);
        double a = calculateAcceleration(targetDistance, elapsedTime);

        // Calculate motor power using position error (Kp), velocity feedback (Kv), and acceleration feedback (Ka)
        double motorPower = (x - currentPosition) * Kp + Kv * v + Ka * a;

        // Limit the motor power to the allowed range (-1 to 1)
        return Math.max(-1, Math.min(1, motorPower));
    }

    // Method to check if the trajectory is done based on position tolerance
    public boolean isTrajectoryDone(double targetDistance, double currentPosition) {
        return Math.abs(targetDistance - currentPosition) < positionTolerance;
    }

    // Method to calculate position based on elapsed time
    private double calculatePosition(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = targetDistance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;
        double cruiseDistance = targetDistance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        if (elapsedTime > entireDt) {
            return targetDistance;  // Reached or passed target distance
        }

        if (elapsedTime < accelerationDt) {
            return 0.5 * maxAcceleration * elapsedTime * elapsedTime;  // Acceleration phase
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            double cruiseCurrentDt = elapsedTime - accelerationDt;
            return accelerationDistance + maxVelocity * cruiseCurrentDt;  // Cruising phase
        } else {
            double decelerationTime = elapsedTime - (accelerationDt + cruiseDt);
            return accelerationDistance + cruiseDistance + maxVelocity * decelerationTime
                    - 0.5 * maxAcceleration * decelerationTime * decelerationTime;  // Deceleration phase
        }
    }

    // Method to calculate velocity at a given elapsed time
    private double calculateVelocity(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = targetDistance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        double cruiseDistance = targetDistance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        if (elapsedTime > entireDt) {
            return 0;  // No velocity after reaching the target
        }

        if (elapsedTime < accelerationDt) {
            return maxAcceleration * elapsedTime;  // Accelerating
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            return maxVelocity;  // Cruising
        } else {
            double decelerationTime = elapsedTime - (accelerationDt + cruiseDt);
            return maxVelocity - maxAcceleration * decelerationTime;  // Decelerating
        }
    }

    // Method to calculate acceleration at a given elapsed time
    private double calculateAcceleration(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = targetDistance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        double cruiseDistance = targetDistance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        if (elapsedTime > entireDt) {
            return 0;  // No acceleration after reaching the target
        }

        if (elapsedTime < accelerationDt) {
            return maxAcceleration;  // Accelerating phase
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            return 0;  // Constant velocity, no acceleration
        } else {
            return -maxAcceleration;  // Decelerating phase
        }
    }
}
