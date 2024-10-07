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
    private PIDController armController;
    private PIDController slideController;
    double armP = 0.004,armI = 0, armD = 0.0002;
    double armF = 0.03;
    int armTarget = 0;
    int armThreshold = 10;
    double armTicksInDegree = 285 / 180;//1425
    double slideP = 0.006,slideI = 0, slideD = 0.0001;
    double slideF = 0.04;
    int slideTarget = 0;
    int slideThreshold = 50;
    double slideTicksInDegree = 358.466 / 180;//1425
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
        IDLE, COMPLETED,MOVING//same as completed
    }
    private armState currentArmState = armState.IDLE;
    private enum StrafeState{
        IDLE,MOVING,COMPLETED
    }
    private StrafeState currentStrafeState = StrafeState.IDLE;
    @Override
    public void init() {
        armController = new PIDController(armP,armI,armD);
        slideController = new PIDController(slideP,slideI,slideD);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor"); // EXP 2
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor"); // EXP 3
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor"); // EXP 0
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor"); // EXP 1
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor"); // CON - 3
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor"); // CON - 0
        wristServo = hardwareMap.get(Servo.class,"wristServo"); // CON - 0
        clawServo = hardwareMap.get(Servo.class,"clawServo"); // EXP - 5
        basketServo = hardwareMap.get(Servo.class,"basketServo"); // CON - 4
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
        clawServo.setPosition(0.75);
        newTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();
        switch(currentOrderState){
            case FIRST:
                if ((currentDriveState == DriveState.IDLE) && (currentSlideState == SlideState.IDLE)){
                    //moveToPos(20,0.2);
                    setTargetSlide(1500);
                    //setTargetArm(3000);
                   // wristServo.setPosition(0.55);
                }
                if ( (currentSlideState == SlideState.COMPLETED)){
                    //currentArmState = armState.IDLE;
                    currentDriveState = DriveState.IDLE;
                    currentSlideState = SlideState.IDLE;
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
        armController.setPID(armP,armI,armD);
        int jointPos = jointMotor.getCurrentPosition();
        double pid = armController.calculate(jointPos,target);
        double ff = Math.cos(Math.toRadians(target / armTicksInDegree)) * armF;
        double power = pid + ff;
        jointMotor.setPower(power);
    }
    public void armFSM(){
        switch (currentArmState){
            case IDLE:
                int currentPos1 = jointMotor.getCurrentPosition();
                moveArm(currentPos1);
                break;
            case COMPLETED:
                moveArm(armTarget);
                break;
            case MOVING:
                moveArm(armTarget);
                double currentPos2 = jointMotor.getCurrentPosition();

                if (Math.abs((armTarget - currentPos2)) < armThreshold){
                    currentArmState = armState.COMPLETED;
                }

        }
    }
    public void setTargetSlide(int target){
        slideTarget = target;
        currentSlideState = SlideState.MOVING;
    }
    public void moveSlide(int target){
        slideController.setPID(slideP,slideI,slideD);
        int slidePos = slideMotor.getCurrentPosition();
        double pid = slideController.calculate(slidePos,target);
        double ff = Math.cos(Math.toRadians(target / slideTicksInDegree)) * slideF;
        double power = pid + ff;
        slideMotor.setPower(power);
    }
    public void slideFSM(){
        switch (currentSlideState){
            case IDLE:
                int currentPos1 = slideMotor.getCurrentPosition();
                moveSlide(currentPos1);
                break;
            case COMPLETED:
                moveSlide(slideTarget);
                break;
            case MOVING:
                moveSlide(slideTarget);
                double currentPos2 = slideMotor.getCurrentPosition();

                if (Math.abs((slideTarget - currentPos2)) < slideThreshold){
                    currentSlideState = SlideState.COMPLETED;
                }

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
    private boolean isDriveState(DriveState currentState, DriveState... targetStates){
        for (DriveState targetState : targetStates){
            if (currentState == targetState){
                return true;
            }
        }
        return false;
    }
    private boolean isArmState(armState currentState, armState... targetStates){
        for (armState targetState : targetStates){
            if (currentState == targetState){
                return true;
            }
        }
        return false;
    }
    private boolean isSlideState(SlideState currentState, SlideState... targetStates){
        for (SlideState targetState : targetStates){
            if (currentState == targetState){
                return true;
            }
        }
        return false;
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
