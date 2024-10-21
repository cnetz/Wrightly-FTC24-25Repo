package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotControl extends OpMode {

    private DcMotorEx motor;
    private MotionProfile motionProfile;
    private int currentStep = 0;  // Tracks the current motion step

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the motion profile
        motionProfile = new MotionProfile();
    }

    public void start() {
        // Start the first motion when the op mode starts
        motionProfile.startMotionProfile();
    }

    // Main control loop
    public void loop() {
        double targetDistance;

        // Sequential motion steps
        switch (currentStep) {
            case 0:
                // First motion: drive 1000 encoder ticks
                targetDistance = 1000;
                drive(targetDistance);

                if (!motor.isBusy()) {
                    currentStep++;  // Move to the next step
                    motionProfile.startMotionProfile();  // Reset the timer for the next motion
                }
                break;

            case 1:
                // Second motion: drive 500 encoder ticks
                targetDistance = 500;
                drive(targetDistance);

                if (!motor.isBusy()) {
                    currentStep++;  // Move to the next step
                    motionProfile.startMotionProfile();  // Reset the timer for the next motion
                }
                break;

            default:
                // All motions are complete, you can add more steps if needed
                break;
        }

        // Telemetry for debugging
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("Current Step", currentStep);
        telemetry.update();
    }

    // Function to drive the motor to a target distance
    public void drive(double targetDistance) {
        double currentPosition = motor.getCurrentPosition();

        // Calculate motor power using the MotionProfile class
        double motorPower = motionProfile.calculateMotorPower(targetDistance, currentPosition);

        // Set motor power
        motor.setPower(motorPower);
    }
}
