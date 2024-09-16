package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleOp.twoWheelDrive;

@Autonomous
public class twoWheelAuto extends OpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 3.77;
    double slideDiameter = 1.5;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);
    double bias = 1.0;
    double conversion = cpi * bias;
    double robotWidth = 12.9;
    double turnCF = Math.PI * robotWidth;
    private DcMotor rightMotor; // location 2
    private DcMotor leftMotor; // location 1
    private DcMotor jointMotor; // location 0
    private DcMotor slideMotor; // location 3
    private Servo wristServo;
    private ElapsedTime newTimer = new ElapsedTime();
    IMU imu;
    private boolean fiveSeconds = false;
    private boolean exit = false;
    private enum OrderState{
        FIRST,SECOND,THIRD
    }
    private OrderState currentOrderState = OrderState.FIRST;
    private enum SlideState{
        IDLE,MOVING,COMPLETED
    }
    private SlideState currentSlideState = SlideState.IDLE;
    private enum DriveState{
        IDLE,MOVING,COMPLETED
    }
    private DriveState currentDriveState = DriveState.IDLE;
    @Override
    public void init() {

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        imu = hardwareMap.get(IMU.class,"imu");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void init_loop() {
        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        } else {
            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        }
        updateTelemetry();
    }

    @Override
    public void start() {
        newTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
       switch(currentOrderState){
           case FIRST:
               moveSlide(12,0.2);
               moveToPos(20,0.2);
               if (currentDriveState == DriveState.COMPLETED && currentSlideState == SlideState.COMPLETED ){
                   currentOrderState = OrderState.SECOND;
               }
               break;
           case SECOND:
               break;
           case THIRD:
               break;
       }
        switch (currentSlideState) {
            case IDLE:
                break;

            case MOVING:
                // Check if the slide has reached the target
                if (!slideMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    currentSlideState = SlideState.COMPLETED;
                }
                break;

            case COMPLETED:
                slideMotor.setPower(0);  // Stop the motor

                // Now transition back to IDLE for future movement
                break;

        }
        //drive state
        switch (currentDriveState) {
            case IDLE:
                break;

            case MOVING:
                // Check if the slide has reached the target
                if (!leftMotor.isBusy() || !rightMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    currentDriveState = DriveState.COMPLETED;
                }
                break;

            case COMPLETED:
                leftMotor.setPower(0);  // Stop the motor
                rightMotor.setPower(0);  // Stop the motor

                // Now transition back to IDLE for future movement
                break;

        }
    }
    @Override
    public void stop() {
        exit = true;
    }

    public void updateTelemetry(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("time", newTimer.seconds());
        telemetry.addData("Slide", slideMotor.getCurrentPosition());
        telemetry.update();
    }

    public void moveToPos(double inches, double speed) {
        exit = false;
        int move = (int)(Math.round(inches * conversion)); // converts inches to cpr for driving forward and back

        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + move);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + move);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        currentDriveState = DriveState.MOVING;

    }
    public void moveSlide(double inches, double speed){
        int move = (int)(Math.round(inches * cpiSlide));
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + move);

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor.setPower(speed);
        currentSlideState = SlideState.MOVING;

    }
    public void moveJoint(double inches, double speed) {
        int move = (int) (Math.round(inches * cpiSlide * 3));
        jointMotor.setTargetPosition(move);

        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jointMotor.setPower(speed);
    }
    public void turnDegrees(double degrees, double speed) {
        double turnDistance = (degrees / 360) * turnCF;
        double targetCounts = conversion * turnDistance;
        leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() + targetCounts));
        rightMotor.setTargetPosition((int) (rightMotor.getCurrentPosition() - targetCounts));

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while(leftMotor.isBusy() && rightMotor.isBusy()){
            if(exit) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                return;
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }
}