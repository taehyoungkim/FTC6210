package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

/**
 * Created by Taehyoung Kim on 10/3/16.
 */
@Autonomous(name="Sensor Test", group = "Testing")
public class LineFollow extends StrykeOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "Initialized.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //telemetry.addData("Left", leftRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Right", rightRange.read8(ModernRoboticsI2cRangeSensor.Register.));

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
