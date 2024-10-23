package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class MotionProfile_CONFIG extends OpMode {
    private DcMotorEx motor;
    // Fixed motion profile parameters as constants
    public static double maxAcceleration = 2.0;  // Acceleration rate (power/second)
    public static double maxVelocity = 2.5;      // Maximum velocity (motor power)
    public static double Kp = 0.00001;             // Proportional constant for position error
    public static double Kv = 0.15;             // Constant for velocity feedback
    public static double Ka = 0.25;            // Constant for acceleration feedback
    public static double target = 0;
    public static double positionTolerance = 100; // Tolerance for stopping motion (encoder ticks)

    // Internal timer to track elapsed time
    private ElapsedTime timer;

    public MotionProfile_CONFIG() {
        timer = new ElapsedTime();
    }

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        motor.setPower(calculateMotorPower(target, motor.getCurrentPosition()));

        telemetry.addData("pos", motor.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.update();
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

        if(Math.abs(targetDistance - currentPosition) < positionTolerance){
            return 0;
        }
        // Limit the motor power to the allowed range (-1 to 1)
        return Math.max(-1, Math.min(1, motorPower));
    }

    // Method to check if the trajectory is done based on position tolerance
    public boolean isTrajectoryDone(double targetDistance, double currentPosition) {
        if(Math.abs(targetDistance - currentPosition) < positionTolerance){
            return true;
        }
        return false;
    }

    // Method to calculate position based on elapsed time
    private double calculatePosition(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = Math.abs(targetDistance) / 2;  // Use absolute value
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;
        double cruiseDistance = Math.abs(targetDistance) - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        double sign = Math.signum(targetDistance);  // Capture the sign of the targetDistance

        if (elapsedTime > entireDt) {
            return targetDistance;  // Reached or passed target distance
        }

        if (elapsedTime < accelerationDt) {
            return sign * 0.5 * maxAcceleration * elapsedTime * elapsedTime;  // Acceleration phase
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            double cruiseCurrentDt = elapsedTime - accelerationDt;
            return sign * (accelerationDistance + maxVelocity * cruiseCurrentDt);  // Cruising phase
        } else {
            double decelerationTime = elapsedTime - (accelerationDt + cruiseDt);
            return sign * (accelerationDistance + cruiseDistance + maxVelocity * decelerationTime
                    - 0.5 * maxAcceleration * decelerationTime * decelerationTime);  // Deceleration phase
        }
    }

    // Method to calculate velocity at a given elapsed time
    private double calculateVelocity(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = Math.abs(targetDistance) / 2;  // Use absolute value
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        double cruiseDistance = Math.abs(targetDistance) - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        double sign = Math.signum(targetDistance);  // Capture the sign of the targetDistance

        if (elapsedTime > entireDt) {
            return 0;  // No velocity after reaching the target
        }

        if (elapsedTime < accelerationDt) {
            return sign * maxAcceleration * elapsedTime;  // Accelerating
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            return sign * maxVelocity;  // Cruising
        } else {
            double decelerationTime = elapsedTime - (accelerationDt + cruiseDt);
            return sign * (maxVelocity - maxAcceleration * decelerationTime);  // Decelerating
        }
    }


    // Method to calculate acceleration at a given elapsed time
    private double calculateAcceleration(double targetDistance, double elapsedTime) {
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = Math.abs(targetDistance) / 2;  // Use absolute value
        double accelerationDistance = 0.5 * maxAcceleration * accelerationDt * accelerationDt;

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        double cruiseDistance = Math.abs(targetDistance) - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double entireDt = accelerationDt + cruiseDt + accelerationDt;

        double sign = Math.signum(targetDistance);  // Capture the sign of the targetDistance

        if (elapsedTime > entireDt) {
            return 0;  // No acceleration after reaching the target
        }

        if (elapsedTime < accelerationDt) {
            return sign * maxAcceleration;  // Accelerating phase
        } else if (elapsedTime < accelerationDt + cruiseDt) {
            return 0;  // Constant velocity, no acceleration
        } else {
            return sign * -maxAcceleration;  // Decelerating phase
        }
    }
}
