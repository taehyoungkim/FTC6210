package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Taehyoung Kim on 10/3/16.
 */
@Autonomous(name="LineFollow Test", group="Linear Op Mode")
public class LineFollow extends StrykeOpMode {
    private OpticalDistanceSensor ods;

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDriveFront = hardwareMap.dcMotor.get("fl");
        rightDriveFront = hardwareMap.dcMotor.get("fr");
        leftDriveBack = hardwareMap.dcMotor.get("bl");
        rightDriveBack = hardwareMap.dcMotor.get("br");

        ods = hardwareMap.opticalDistanceSensor.get("ods");

        waitForStart();

        while(opModeIsActive()) {
            double value = ods.getLightDetected();
            while (value < 0.25) {
                setDriveSpeed(0.5, 0.5);
            }
            if(value >= 0.25) {
                setRightDriveSpeed(-0.2);
                setLeftDriveSpeed(0);
            } else {
                setLeftDriveSpeed(-0.2);
                setRightDriveSpeed(0);
            }
            telemetry.addData("ODS: ", value);
            telemetry.update();
            idle();
        }
    }
}
