package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class newBotAuto extends OpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 3.77;

    double slideDiameter = 1.5;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);
    double bias = 1.0;
    double conversion = cpi * bias;
    private DcMotor middleMotor; // location 0 - eh
    private DcMotor frontLeftMotor; // location 0 - ch
    private DcMotor frontRightMotor; // location 1 - ch
    private DcMotor backRightMotor; // location 2 - ch
    private DcMotor backLeftMotor; // location 3 - ch
    private DcMotor jointMotor; // location 1 - eh
    private DcMotor slideMotor; // location 2 - eh
    private Servo claw;
    private ElapsedTime newTimer = new ElapsedTime();
    private DigitalChannel green0; //0
    private DigitalChannel red0; //1
    private RevBlinkinLedDriver led;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private RevColorSensorV3 colorSensor;
    IMU imu;
    private boolean fiveSeconds = false;
    private boolean exit = false;
    double greenColor = 0;
    double blueColor = 0;
    double redColor = 0;
    double alphaColor = 0;
    @Override
    public void init() {

        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        claw = hardwareMap.get(Servo.class,"claw");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
        imu = hardwareMap.get(IMU.class,"imu");

        green0 = hardwareMap.get(DigitalChannel.class, "green0");
        red0 = hardwareMap.get(DigitalChannel.class, "red0");

        green0.setMode(DigitalChannel.Mode.INPUT);
        red0.setMode(DigitalChannel.Mode.INPUT);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor.enableLed(true);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

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
        updateColor();
        updateTelemetry();
    }

    @Override
    public void start() {
        newTimer.reset();
        moveToPos(12, 0.2);
        strafeToPos(12, 0.2);
        strafeToPos(-12, 0.2);
        moveToPos(-12, 0.2);
    }

    @Override
    public void loop() {
        updateColor();
        updateTelemetry();
        /*
        if(blueColor > 2750) {
            strafeToPos(12, 0.2);

        }

        if(redColor > 1750) {
            strafeToPos(-12, 0.2);
        }

         */

        /*
        if(blueColor > 2000){
            stopMoving();
            strafeToPos(12, 0.2);
        }

        if (!fiveSeconds && newTimer.seconds() >= 5){
           // moveToPos(36,0.2);
            green0.setState(true);
            red0.setState(false);
            telemetry.addData("something","test");
            telemetry.update();
            fiveSeconds = true;
        }else {
            moveToPos(12, 0.2);
            moveToPos(-12, 0.2);
        }
         */
    }
    @Override
    public void stop() {
        exit = true;
        stopMoving();
    }

    public void updateTelemetry(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("time", newTimer.seconds());
        telemetry.addData("blueColor", blueColor);
        telemetry.addData("alphaColor", alphaColor);
        telemetry.addData("greenColor", greenColor);
        telemetry.addData("redColor", redColor);
        telemetry.addData("Slide", slideMotor.getCurrentPosition());
        telemetry.update();
    }

    public void updateColor() {
        greenColor = colorSensor.green();
        blueColor = colorSensor.blue();
        redColor = colorSensor.red();
        alphaColor = colorSensor.alpha();
    }

    public void moveToPos(double inches, double speed) {
        exit = false;
        int move = (int)(Math.round(inches * conversion)); // converts inches to cpr for driving forward and back

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);//sets position to go to
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

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()){
            if(exit) {
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                return;
            }
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void strafeToPos(double inches, double speed) {
        exit = false;
        int move = (int)(Math.round(inches * conversion));

        middleMotor.setTargetPosition(middleMotor.getCurrentPosition() + move);

        middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        middleMotor.setPower(speed);

        while(middleMotor.isBusy()){
            if (exit) {
                middleMotor.setPower(0);
                return;
            }
        }

        middleMotor.setPower(0);
    }

    public void stopMoving() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        middleMotor.setPower(0);
    }
    public void moveSlide(double inches, double speed){
        int move = (int)(Math.round(inches * cpiSlide));
        slideMotor.setTargetPosition(move);

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor.setPower(speed);

    }
    public void moveJoint(double inches, double speed) {
        int move = (int) (Math.round(inches * cpiSlide * 3));
        jointMotor.setTargetPosition(move);

        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jointMotor.setPower(speed);
    }






}