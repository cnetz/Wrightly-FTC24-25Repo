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
public class twoWheelDrive extends LinearOpMode {
    private DcMotor jointMotor; // location 0
    private DcMotor leftMotor; // location 1
    private DcMotor rightMotor; // location 2
    private DcMotor slideMotor; // location 3
    private Servo wristServo; // location 0
    private Servo clawServo; // location 2
    private Servo basketServo; // location 4
    private ElapsedTime newTimer = new ElapsedTime();
    private enum SlideState {
        COMPLETED,
        IDLE,
        MANUAL,
        MOVING
    }
    private SlideState currentSlideState = SlideState.IDLE;

    @Override
    public void runOpMode()  {

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        basketServo = hardwareMap.get(Servo.class,"basketServo");

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reverse the other motors and sex X to not negative

        double cpr = 537.7;
        double gearRatio = 1;
        double slideDiameter = 1.5;
        double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);

        double changeSpeed = 0.65;
        boolean changeSpeedPos = false;
        boolean wristPos = false;
        boolean clawPos = false;
        boolean wristPosX = false;
        boolean rbPos = false;

        waitForStart();
        //wristServo.setPosition(0.45);

        newTimer.reset();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y;// set y to gamepad 1 left stick y
            double x = gamepad1.left_stick_x;// set x to gamepad 1 left stick x

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);//maxumum x+y =1
            double rightMotorPower = ((y - x) / denominator);
            double leftMotorPower = ((y + x) / denominator);// front left motor power = y+x/den

            rightMotor.setPower(rightMotorPower * changeSpeed);//back right motor it sets power/change speed
            leftMotor.setPower(leftMotorPower * changeSpeed);//for back left  motor it  sets power/change speed

            //slideMotor.setPower(gamepad2.left_stick_y)
            if (gamepad2.right_trigger >0){
                jointMotor.setPower(gamepad2.right_trigger);
            } else if(gamepad2.left_trigger >0) {
                jointMotor.setPower(-gamepad2.left_trigger);
            }else{
                jointMotor.setPower(0);
            }

            switch (currentSlideState) {
                case IDLE:
                    if(gamepad2.left_stick_y != 0){
                        currentSlideState = SlideState.MANUAL;
                    } else if(gamepad2.dpad_down) {
                        int move = (int)(Math.round(-20 * cpiSlide));
                        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + move);

                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        slideMotor.setPower(0.2);

                        currentSlideState = SlideState.MOVING;
                    }
                    break;

                case MOVING:
                    // Check if the slide has reached the target
                    if (!slideMotor.isBusy()) {
                        // If it's done moving, transition to COMPLETED state
                        currentSlideState = SlideState.COMPLETED;
                    }
                    break;

                case COMPLETED:
                    // Stop the motor and set back to RUN_USING_ENCODER for manual control
                    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slideMotor.setPower(0);  // Stop the motor

                    // Now transition back to IDLE for future movement
                    currentSlideState = SlideState.IDLE;
                    break;

                case MANUAL:
                    slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slideMotor.setPower(gamepad2.left_stick_y);
                    // If no manual control is happening, return to IDLE
                    if (Math.abs(gamepad2.left_stick_y) == 0) {
                        currentSlideState = SlideState.IDLE;
                    }
                    break;
            }

            if (buttonHandler.isPressedOnceA_1(gamepad1.a)) {
                if (changeSpeedPos) {
                    changeSpeedPos = false;
                    changeSpeed = 0.65; //speed is 1

                } else { //speed is halved
                    changeSpeed = 0.25 ;
                    changeSpeedPos = true;
                }
            }
            if (buttonHandler.isPressedOnceB_2(gamepad2.b)) {
                if (wristPos) {
                    wristServo.setPosition(0.45);
                    wristPos = false;

                } else {
                    wristPos = true;
                    wristServo.setPosition(0.9);
                }
            }
            if (buttonHandler.isPressedOnceA_2(gamepad2.a)) {
                if (clawPos) {
                    clawServo.setPosition(.1);
                    clawPos = false;

                } else {
                    clawPos = true;
                    clawServo.setPosition(0.75);
                }
            }
            if (buttonHandler.isPressedOnceX_2(gamepad2.x)) {
                if (wristPosX) {
                    wristServo.setPosition(0.45);
                    wristPosX = false;

                } else {
                    wristPosX = true;
                    wristServo.setPosition(0.1);
                }
            }
            if (gamepad2.y) {
                clawServo.setPosition(0.2);
            }  else {
                clawServo.setPosition(0.7);
            }
            if (buttonHandler.isPressedOnceRB_2(gamepad2.right_bumper)) {
                if (rbPos) {
                    basketServo.setPosition(0.45);
                    rbPos = false;

                } else {
                    rbPos = true;
                    basketServo.setPosition(0.1);
                }
            }


            telemetry.addData("Slide State", currentSlideState);
            telemetry.addData("time", newTimer.seconds());// tells us the time from when we press start
            telemetry.addData("jointMotorPos", jointMotor.getCurrentPosition());
            telemetry.addData("slideMotorPos", slideMotor.getCurrentPosition());
            telemetry.addData("wristServo", wristServo.getPosition());
            telemetry.update();
        }
    }
}
