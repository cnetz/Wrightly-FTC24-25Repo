package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class newBotAuto extends OpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 3.77;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.7;
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
    private boolean fiveSeconds = false;
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

        green0 = hardwareMap.get(DigitalChannel.class, "green0");
        red0 = hardwareMap.get(DigitalChannel.class, "red0");

        green0.setMode(DigitalChannel.Mode.INPUT);
        red0.setMode(DigitalChannel.Mode.INPUT);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor.enableLed(true);
    }
    @Override
    public void start() {

        newTimer.reset();
        moveToPos(12, 0.2);
        strafeToPos(12, 0.2);
    }

    @Override
    public void loop() {
        double greenColor = colorSensor.green();
        double blueColor = colorSensor.blue();
        double redColor = colorSensor.red();
        double alphaColor = colorSensor.alpha();

        if (!fiveSeconds && newTimer.seconds() >= 5){
           // moveToPos(36,0.2);
            green0.setState(true);
            red0.setState(false);
            telemetry.addData("something","test");
            telemetry.update();
            fiveSeconds = true;
        }

        telemetry.addData("time", newTimer.seconds());
        telemetry.addData("blueColor", blueColor);
        telemetry.addData("alphaColor", alphaColor);
        telemetry.addData("greenColor", greenColor);
        telemetry.addData("redColor", redColor);
        telemetry.update();
    }
    @Override
    public void stop() {
    }

    public void moveToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()){

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }
    public void strafeToPos(double inches, double speed) {
        int move = (int)(Math.round(inches * conversion));

        middleMotor.setTargetPosition(middleMotor.getCurrentPosition() - move);

        middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        middleMotor.setPower(speed);

        while(middleMotor.isBusy()){

        }

        middleMotor.setPower(0);


    }





}