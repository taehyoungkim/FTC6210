package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Taehyoung Kim on 10/3/16.
 */
@Autonomous(name="LineFollow Test", group = "Testing")
public class LineFollow extends StrykeOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "Initialized.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            double value = ods.getLightDetected();
            if (value < 0.5) {
                setDriveSpeed(0.5, -0.5);
            } else searchForLine(0.3,500,true);

            telemetry.addData("ODS: ", value);
            telemetry.update();
            idle();
        }
    }

    public void searchForLine(double speed, long ms, boolean left) throws InterruptedException {

        long nextTime = System.currentTimeMillis() + ms;
        while(ods.getLightDetected() < 0.5) {
            if(nextTime <= System.currentTimeMillis()) {
                left = !left;
                nextTime = (long) (System.currentTimeMillis() + (ms*1.3));
                continue;
            }
            if(left) {
                setDriveSpeed(-speed, -speed);
            } else {
                setDriveSpeed(speed, speed);
            }
            idle();
        }
    }
}
