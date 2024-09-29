package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleOp.twoWheelDrive;

@Autonomous
public class twoWheelAuto extends OpMode {
    private PIDController controller;
    double p = 0.004,i = 0, d = 0.0002;
    double f = 0.03;
    int armTarget = 0;
    int armThreshold = 10;
    double ticksInDegree = 285 / 180;//1425
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
    private DcMotorEx jointMotor; // location 0
    private DcMotor slideMotor; // location 3
    private Servo wristServo;
    private Servo clawServo;
    private Servo basketServo;
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
    private enum armState{
        IDLE, HOLDING,MOVING//same as completed
    }
    private armState currentArmState = armState.IDLE;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        basketServo = hardwareMap.get(Servo.class,"basketServo");
        imu = hardwareMap.get(IMU.class,"imu");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        clawServo.setPosition(0.75);
        newTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
        switch(currentOrderState){
            case FIRST:
                if (currentArmState == armState.IDLE){
                    setTargetArm(3000);
                    wristServo.setPosition(0.55);
                }
                if (currentArmState == armState.HOLDING){
                    currentArmState = armState.IDLE;
                    currentOrderState = OrderState.SECOND;
                }
                /*if (currentDriveState == DriveState.IDLE && currentSlideState == SlideState.IDLE){
                //moveSlide(12,0.2);
                // moveToPos(20,0.2);
            }
                if (currentDriveState == DriveState.COMPLETED && currentSlideState == SlideState.COMPLETED){
                    currentDriveState = DriveState.IDLE;
                    currentSlideState = SlideState.IDLE;
                    currentOrderState = OrderState.SECOND;
                }*/
                break;
            case SECOND:
                if (currentDriveState == DriveState.IDLE && currentSlideState == SlideState.IDLE){
                   // moveSlide(-12,0.2);
                   // moveToPos(-20,0.2);
                }
                if (currentDriveState == DriveState.COMPLETED && currentSlideState == SlideState.COMPLETED){
                    currentDriveState = DriveState.IDLE;
                    currentSlideState = SlideState.IDLE;
                    currentOrderState = OrderState.THIRD;
                }
                break;
            case THIRD:
                break;
        }

        driveFSM();
        armFSM();
        slideFSM();
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
        telemetry.addData("DriveLeft", leftMotor.getCurrentPosition());
        telemetry.addData("DriveRight", rightMotor.getCurrentPosition());
        telemetry.addData("DriveState", currentDriveState);
        telemetry.addData("SlideState", currentSlideState);
        telemetry.addData("ArmState", currentArmState);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("jointPos", jointMotor.getCurrentPosition());
        telemetry.addData("jointPower", jointMotor.getPower());
        telemetry.update();
    }

    public void moveToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

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
    public void setTargetArm(int target){
        armTarget = target;
        currentArmState = armState.MOVING;
    }
    public void moveArm(int target){
        controller.setPID(p,i,d);
        int jointPos = jointMotor.getCurrentPosition();
        double pid = controller.calculate(jointPos,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;
        jointMotor.setPower(power);
    }
    public void armFSM(){
        switch (currentArmState){
            case IDLE:
                int currentPos1 = jointMotor.getCurrentPosition();
                moveArm(currentPos1);
                break;
            case HOLDING:
                moveArm(armTarget);
                break;
            case MOVING:
                moveArm(armTarget);
                double currentPos2 = jointMotor.getCurrentPosition();

                if (Math.abs((armTarget - currentPos2)) < armThreshold){
                    currentArmState = armState.HOLDING;
                }

        }
    }
    public void slideFSM(){
        switch (currentSlideState) {
            case IDLE:
                break;

            case MOVING:
                // Check if the slide has reached the target
                if (!slideMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    slideMotor.setPower(0);  // Stop the motor
                    currentSlideState = SlideState.COMPLETED;
                    break;
                }
                break;

            case COMPLETED:
                // Now transition back to IDLE for future movement
                break;

        }

    }
    public void driveFSM(){
        switch (currentDriveState) {
            case IDLE:
                break;

            case MOVING:
                // converts inches to cpr for driving forward and back
                // Check if the slide has reached the target
                if (!leftMotor.isBusy() && !rightMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    leftMotor.setPower(0);  // Stop the motor
                    rightMotor.setPower(0);  // Stop the motor

                    currentDriveState = DriveState.COMPLETED;
                    break;
                }
                break;

            case COMPLETED:
                // Now transition back to IDLE for future movement
                break;
        }

    }
    //private double calculateSinusoidalSpeed(int currentDistance, int totalDistance)
    //fractionTraveled = (double) currentDistance / totalDistance;
    //speed = baseSpeed + (maxSpeed - baseSpeed) * Math.sin(Math.PI * fractionTraveled);
    //return speed;

    //private double applyAccelerationControl(double currentSpeed, double targetSpeed, double maxAcceleration)
    // speedDifference = targetSpeed - currentSpeed
    // if(speedDifference > maxAcceleration) {
    // currentSpeed += maxAcceleration
    // } else if (speedDifference < -maxAcceleration) {
    // currentSpeed -= maxAcceleration
    // } else {
    // currentSpeed = targetSpeed;
    // }
    // return currentSpeed;

}
