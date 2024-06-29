package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class PressedTest extends LinearOpMode {
    //hardwaremap here
    @Override
    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            boolean gamepad1A_pressed = gamepad1.a;

<<<<<<< Updated upstream
            if (gamepad1A_pressed & !pressedLastIteration) {
                //DO STUFF
            }

            pressedLastIteration = gamepad1A_pressed;


=======
            if (buttonHandler.isPressedOnceA(gamepad1A_pressed)) {
                telemetry.addLine("Gampad 1 Pressed");
                telemetry.update();
            }
>>>>>>> Stashed changes

        }

    }
}
