package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private DcMotor jointMotor; // location 0
    private DcMotor leftMotor; // location 1
    private DcMotor rightMotor; // location 2
    private DcMotor slideMotor; // location 3
    private Servo wristServo; // location 0
    private Servo clawServo; // location 2
    private HardwareMap hardwareMap;
    public enum Mode {
        FLOAT,
        BRAKE
    }
    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }
    public void initHardware(){
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorMode(Mode mode){
        if (mode == Mode.BRAKE) {
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
