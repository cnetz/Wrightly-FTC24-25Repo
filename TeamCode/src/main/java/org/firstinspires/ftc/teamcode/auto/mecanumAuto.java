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
public class mecanumAuto extends OpMode {
    private PIDController controller;
    double p = 0.004,i = 0, d = 0.0002;
    double f = 0.03;
    int armTarget = 0;
    int armThreshold = 10;
    double ticksInDegree = 285 / 180;//1425
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 4.1; //4.09449 in inches - mecanum
    double slideDiameter = 1.5;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);
    double bias = 1.0;
    double conversion = cpi * bias;
    double robotWidth = 12.9;
    double turnCF = Math.PI * robotWidth;
    private DcMotor jointMotor; // location
    private DcMotor frontLeftMotor; // location
    private DcMotor backLeftMotor; // location
    private DcMotor backRightMotor; // location
    private DcMotor frontRightMotor; // location
    private DcMotor slideMotor; // location
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
    private enum StrafeState{
        IDLE,MOVING,COMPLETED
    }
    private StrafeState currentStrafeState = StrafeState.IDLE;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        basketServo = hardwareMap.get(Servo.class,"basketServo");
        imu = hardwareMap.get(IMU.class,"imu");

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        strafeFSM();
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
        telemetry.addData("frontLeft", frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontRight", frontRightMotor.getCurrentPosition());
        telemetry.addData("backLeft", backLeftMotor.getCurrentPosition());
        telemetry.addData("backRight", backRightMotor.getCurrentPosition());
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

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        currentDriveState = DriveState.MOVING;

    }
    public void strafeToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        currentStrafeState = StrafeState.MOVING;

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
                if (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    currentDriveState = DriveState.COMPLETED;
                    break;
                }
                break;

            case COMPLETED:
                // Now transition back to IDLE for future movement
                break;
        }

    }
    public void strafeFSM() {
        switch (currentStrafeState) {
            case IDLE:
                break;

            case MOVING:
                // converts inches to cpr for driving forward and back
                // Check if the slide has reached the target
                if (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    currentStrafeState = StrafeState.COMPLETED;
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
