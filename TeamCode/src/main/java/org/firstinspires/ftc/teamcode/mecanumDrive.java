package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.mecanumAuto;
import org.firstinspires.ftc.teamcode.teleOp.twoWheelDrive;

@TeleOp
public class mecanumDrive extends LinearOpMode {
    double cpr = 537.7;
    double gearRatio = 1;
    double slideDiameter = 1.5;
    double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);
    double bias = 1.1;
    private PIDController controller;
    public static double p = 0.004,i = 0, d = 0.0002;
    public static double f = 0.03;
    private final double ticksInDegree = 285 / 180;//1425
    int armTarget = 0;
    private DcMotor jointMotor; // location
    private DcMotor frontLeftMotor; // location
    private DcMotor backLeftMotor; // location
    private DcMotor backRightMotor; // location
    private DcMotor frontRightMotor; // location
    private DcMotor slideMotor; // location
    private Servo wristServo; // location
    private Servo clawServo; // location
    private Servo basketServo; // location

    private ElapsedTime newTimer = new ElapsedTime();
    private enum armState{
        IDLE, HOLDING,MOVING//same as completed
    }
    private armState currentArmState = armState.IDLE;
    private enum SlideState {
        COMPLETED,
        IDLE,
        MANUAL,
        MOVING
    }
    private SlideState currentSlideState = SlideState.IDLE;

    @Override
    public void runOpMode()  {
        controller = new PIDController(p,i,d);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        basketServo = hardwareMap.get(Servo.class,"basketServo");

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reverse the other motors and sex X to not negative

        double changeSpeed = 0.65;
        boolean changeSpeedPos = false;
        boolean wristPos = false;
        boolean clawPos = false;
        boolean wristPosX = false;
        boolean rbPos = false;

        int armTempPos = 0;

        waitForStart();
        //wristServo.setPosition(0.45);
        armTarget = jointMotor.getCurrentPosition();
        clawServo.setPosition(0.6);

        newTimer.reset();

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            double y = gamepad1.left_stick_y;// set y to gamepad 1 left stick y
            double x = -gamepad1.left_stick_x * bias;// set x to gamepad 1 left stick x
            double rx = -gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);//maxumum x+y =1
            double backRightPower = ((y + x - rx) / denominator);
            double frontLeftPower = ((y + x + rx) / denominator);// front left motor power = y+x/den
            double frontRightPower = ((y - x - rx) / denominator);
            double backLeftPower = ((y - x + rx) / denominator);// front left motor power = y+x/den

            frontRightMotor.setPower(frontRightPower * changeSpeed);//back right motor it sets power/change speed
            backRightMotor.setPower(backRightPower * changeSpeed);//back right motor it sets power/change speed
            frontLeftMotor.setPower(frontLeftPower * changeSpeed);//back right motor it sets power/change speed
            backLeftMotor.setPower(backLeftPower * changeSpeed);//for back left  motor it  sets power/change speed

            //slideMotor.setPower(gamepad2.left_stick_y)
            if (gamepad2.right_trigger > 0){
                armTempPos = armTarget + 5;
                setTargetArm(armTempPos);
            } else if(gamepad2.left_trigger >0) {
                armTempPos = armTarget - 5;
                setTargetArm(armTempPos);
            }else{
                currentArmState = armState.HOLDING;
            }

            armFSM();
            slideFSM();

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
                    wristServo.setPosition(0.75);
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
                clawServo.setPosition(0.6);
            }  else {
                clawServo.setPosition(0.75); //if claw loose adjust + 0.05
            }
            if (buttonHandler.isPressedOnceRB_2(gamepad2.right_bumper)) {
                if (rbPos) {
                    basketServo.setPosition(0.7);
                    rbPos = false;

                } else {
                    rbPos = true;
                    basketServo.setPosition(0.3);
                }
            }
            telemetry.addData("arm State", currentArmState);
            telemetry.addData("target ", armTarget);
            telemetry.addData("time", newTimer.seconds());// tells us the time from when we press start
            telemetry.addData("jointMotorPos", jointMotor.getCurrentPosition());
            telemetry.addData("slideMotorPos", slideMotor.getCurrentPosition());
            telemetry.addData("wristServo", wristServo.getPosition());
            telemetry.update();
        }
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
        }
    }
    public void slideFSM(){
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
    }
}



