'org.firstinspires.ftc.teamcode.teleOp';
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColorSensorExample extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        HardwareMap hardwareMap = hardwareMap;
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        // Wait for the start button to be pressed
        waitForStart();

        // Run detection loop
        while (opModeIsActive()) {
            // Read color sensor data
            int redValue = colorSensor.red();
            int blueValue = colorSensor.blue();

            // Define thresholds for red and blue detection
            int redThreshold = 50;  // Adjust this based on your sensor's readings
            int blueThreshold = 50; // Adjust this based on your sensor's readings

            // Check if red is detected
            if (redValue > redThreshold && redValue > blueValue) {
                telemetry.addData("Status", "Red detected");
                telemetry.update();
            }

            // Check if blue is detected
            if (blueValue > blueThreshold && blueValue > redValue) {
                telemetry.addData("Status", "Blue detected");
                telemetry.update();
            }

            // Delay to avoid updating telemetry too frequently
            sleep(100);
        }
    }
}










