package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class RobotControl extends OpMode {

    private DcMotorEx motor;
    private MotionProfile motionProfile;
    private int currentStep = 0;  // Tracks the current motion step

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the motion profile
        motionProfile = new MotionProfile();
    }

    public void init_loop() {
        telemetry.addData("CurrentPos", motor.getCurrentPosition());
        telemetry.addData("Current Step", currentStep);
        telemetry.update();
    }
    public void start() {
        // Start the first motion when the op mode starts
        //motionProfile.startMotionProfile();
    }

    // Main control loop
    public void loop() {
        // Sequential motion steps
        switch (currentStep) {
            case 0:
                drive(1000);

                if (motionProfile.isTrajectoryDone(1000, motor.getCurrentPosition())) {
                    //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentStep++;  // Move to the next step
                    //motionProfile.startMotionProfile();  // Reset the timer for the next motion
                }
                break;
            case 1:
                drive(-1000);

                if (motionProfile.isTrajectoryDone(-1000, motor.getCurrentPosition())) {
                    //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentStep++;  // Move to the next step
                    //motionProfile.startMotionProfile();  // Reset the timer for the next motion
                }
            case 2:
                telemetry.addLine("COMPLETED");
                break;
            default:
                // All motions are complete, you can add more steps if needed
                break;
        }

        // Telemetry for debugging
        telemetry.addData("CurrentPos", motor.getCurrentPosition());
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

        telemetry.addData("Target Distance", targetDistance);
        telemetry.addData("Calculated Motor Power", motorPower);
    }
}
