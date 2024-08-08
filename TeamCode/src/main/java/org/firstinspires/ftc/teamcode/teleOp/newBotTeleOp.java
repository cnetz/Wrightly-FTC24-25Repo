package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ButtonHandler;

@TeleOp
public class newBotTeleOp extends LinearOpMode {
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


    @Override
    public void runOpMode()  {

        middleMotor = hardwareMap.get(DcMotor.class, "middleMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        claw = hardwareMap.get(Servo.class,"claw");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        green0 = hardwareMap.get(DigitalChannel.class, "green0");
        red0 = hardwareMap.get(DigitalChannel.class, "red0");

        green0.setMode(DigitalChannel.Mode.INPUT);
        red0.setMode(DigitalChannel.Mode.INPUT);
        
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Reverse the other motors and sex X to not negative

        float defaultPower = 2;
        double changeSpeed = 1;
        double middleDefaultPower = 1.25;
        double slide = 0;
        double joint = 0;

        boolean servoFirstPos = true;
        boolean servoMiddlePos = false;
        boolean servoLastPos = false;

        boolean changeSpeedPos = false;


        waitForStart();

        newTimer.reset();

        claw.setPosition(0.3);//claw = intake

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y;// set y to gamepad 1 left stick y
            double x = gamepad1.left_stick_x;// set x to gamepad 1 left stick x
            double rx = -gamepad1.right_stick_x;// sets rotating to gamepad 1 right stick

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);//maxumum x+y =1
            double middleMotorPower = ((rx)) / middleDefaultPower;// sets middle motor power to rx / default
            double frontLeftMotorPower = ((y + x) / denominator) / defaultPower;// front left motor power = y+x/den/defoult
            double backLeftMotorPower = ((y + x) / denominator) / defaultPower;// back left motor power = y+x/den/defoult
            double frontRightMotorPower = ((y - x) / denominator) / defaultPower;// front right  motor power = y-x/den/defoult
            double backRightMotorPower = ((y - x) / denominator) / defaultPower;// back right motor power = y-x/den/defoult

            middleMotor.setPower(middleMotorPower / changeSpeed);//for middle motor it sets power/change speed
            frontLeftMotor.setPower(frontLeftMotorPower / changeSpeed);//for front left motor itsets power/change speed
            frontRightMotor.setPower(frontRightMotorPower / changeSpeed);//for front right  motor it sets power/change speed
            backRightMotor.setPower(backRightMotorPower / changeSpeed);//back right motor it sets power/change speed
            backLeftMotor.setPower(backLeftMotorPower / changeSpeed);//for back left  motor it  sets power/change speed

            if (gamepad2.right_trigger > 0){
                slide = gamepad2.right_trigger * 4;
            } else if (gamepad2.left_trigger > 0){
                slide = -gamepad2.left_trigger / 1.5;
            } else {
                slide = 0;
            }

            slideMotor.setPower(slide);

            if (gamepad1.right_trigger > 0){
                joint = gamepad1.right_trigger * 4;
            } else if (gamepad1.left_trigger > 0){
                joint = -gamepad1.left_trigger / 1.5;
            } else {
                joint = 0;
            }
            jointMotor.setPower(joint);

            boolean gamepad1A_pressed = gamepad1.a;
            boolean gamepad1B_pressed = gamepad1.b;

            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                if (changeSpeedPos){
                    changeSpeedPos = false;
                    changeSpeed = 1;

                } else {
                    changeSpeed = 2;
                    changeSpeedPos = true;
                }
            }
            if (buttonHandler.isPressedOnceB(gamepad1B_pressed)) {
                telemetry.addData("B", gamepad1B_pressed);
            }
            if (gamepad2.dpad_down) {
                jointMotor.setTargetPosition(410);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(0.25);
            }
            if (gamepad2.dpad_up) {
                jointMotor.setTargetPosition(0);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(0.25);
            }
            if (gamepad2.dpad_right) {
                jointMotor.setTargetPosition(-50);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(0.25);
            }
            if (newTimer.seconds() >= 15){
                telemetry.addLine("15");

                green0.setMode(DigitalChannel.Mode.INPUT);
                red0.setMode(DigitalChannel.Mode.OUTPUT);
                //pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

            }else if (newTimer.seconds() >= 5){
                green0.setMode(DigitalChannel.Mode.OUTPUT);
                red0.setMode(DigitalChannel.Mode.INPUT);

                //pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                telemetry.addLine("5");

            }else{

                telemetry.addLine("0");
                
            }

            //led.setPattern(pattern);

            telemetry.addData("X Value", x);// lets us know the x value
            telemetry.addData("time", newTimer.seconds());// tells us the time from when we press start
            telemetry.addData("slide", slide);
            telemetry.addData("jointMotorpos", jointMotor.getCurrentPosition());
            telemetry.addData("jointMotor",joint);
            telemetry.update();

        }

    }
}
