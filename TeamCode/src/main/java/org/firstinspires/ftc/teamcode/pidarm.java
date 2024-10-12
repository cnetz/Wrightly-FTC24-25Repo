package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class pidarm extends OpMode {
    private PIDController controller;
    public static double p = 0.006,i = 0, d = 0.0001;
    public static double f = 0.04;
    public static int target = 0;
    private final double ticksInDegree = 358.466 / 180;//1425
    private DcMotorEx slideMotor;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int slidePos = slideMotor.getCurrentPosition();
        double pid = controller.calculate(slidePos,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;
        slideMotor.setPower(power);

        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
