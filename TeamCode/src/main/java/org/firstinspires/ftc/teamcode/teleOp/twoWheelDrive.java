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

    private ElapsedTime newTimer = new ElapsedTime();

    @Override
    public void runOpMode()  {

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Reverse the other motors and sex X to not negative

        double changeSpeed = 1;
        double slide = 0;
        double joint = 0;
        boolean changeSpeedPos = false;
        boolean wristPos = false;
        boolean clawPos = false;
        waitForStart();

       // newTimer.reset();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y;// set y to gamepad 1 left stick y
            double x = gamepad1.left_stick_x;// set x to gamepad 1 left stick x

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);//maxumum x+y =1
            double rightMotorPower = ((y - x) / denominator);
            double leftMotorPower = ((y + x) / denominator);// front left motor power = y+x/den

            rightMotor.setPower(rightMotorPower / changeSpeed);//back right motor it sets power/change speed
            leftMotor.setPower(leftMotorPower / changeSpeed);//for back left  motor it  sets power/change speed

            slideMotor.setPower(gamepad2.left_stick_y);
            jointMotor.setPower(gamepad2.right_stick_y);

            if (buttonHandler.isPressedOnceA_1(gamepad1.a)) {
                if (changeSpeedPos){
                    changeSpeedPos = false;
                    changeSpeed = 1;

                } else {
                    changeSpeed = 2;
                    changeSpeedPos = true;
                }
            }
            if (buttonHandler.isPressedOnceB_2(gamepad2.b)) {
                if (wristPos){
                    wristServo.setPosition(0.65);
                    wristPos = false;

                } else {
                    wristPos = true;
                    wristServo.setPosition(0.2);
                }
            }
            if (buttonHandler.isPressedOnceA_2(gamepad2.a)) {
                if (clawPos){
                    clawServo.setPosition(0);
                    clawPos = false;

                } else {
                    clawPos = true;
                    clawServo.setPosition(0.25);
                }
            }

            telemetry.addData("X Value", x);// lets us know the x value
            telemetry.addData("time", newTimer.seconds());// tells us the time from when we press start
            telemetry.addData("slide", slide);
            telemetry.addData("jointMotorpos", jointMotor.getCurrentPosition());
            telemetry.addData("jointMotor",joint);
            telemetry.update();

        }

    }
}
