package org.firstinspires.ftc.teamcode.stryke.auto;

import android.app.Activity;
import android.content.Context;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;


@Autonomous(name = "Main RED Auto") //GEN3 FIELD
public class MainRedAuto extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        getGyro().calibrate();
        //calibrateGyro();

        double speed = speedFromVoltage();
        // equation derived from above
        //double speed = 7/25 * voltage + 2041/500;
        telemetry.addData("Speed", speed);
        telemetry.addData("Status", "Ready!");
        telemetry.addData("Encoder", (Math.abs(leftDrive1.getCurrentPosition()) + Math.abs(rightDrive1.getCurrentPosition()))/2 + "");
        telemetry.update();

        waitForStart();
        //getGyro().resetZAxisIntegrator();
        simpleWait(200);
        runtime.reset();

        encoderDriveBETA(15 , 0.7, 100000,getDriveMotors());

        double heading = getGyro().getHeading();
        setDriveSpeed(-1,-1);
        while((heading > 360 - 35 || heading < 90) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }
        stopDriveMotors();
        simpleWait(100);

        goToFirstBeacon();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }
        simpleWait(100);

        mashBeacon(false);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }
        simpleWait(100);

        goToSecondBeacon();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }
        mashBeacon(false);
        simpleWait(100);

        heading = getGyro().getHeading();
        setDriveSpeed(-1, -1);
        while (heading > 165) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        stopDriveMotors();

        encoderDriveBETA(26 ,0.6, 1000, getDriveMotors());
        shootTwoBalls();
        manipulator.setPower(-0.5);

        
        encoderDriveBETA(25, 0.6, 5000, getDriveMotors());
        manipulator.setPower(0);

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    private void goToSecondBeacon() throws InterruptedException {
        double heading = getGyro().getHeading();
        double start = heading;
        int count = 0;
        long failSafe = System.currentTimeMillis() + 500;
        setDriveSpeed(1, 1);
        while( heading < 350 && heading > 90 && opModeIsActive()){
            heading = getGyro().getHeading();
//            if(start == heading) count ++;
//            if(count >= 1000 && System.currentTimeMillis() > failSafe) requestOpModeStop();
            telemetry.addData("Heading", heading);
            telemetry.update();
            if(isStopRequested()) return;
        }
        stopDriveMotors();

        encoderDriveBETA(24 * 3 - 23, 0.6 ,1000, getDriveMotors());

        setDriveSpeed(-1, -1);
        while((heading > 286 || heading < 90) && opModeIsActive()) {
            if(isStopRequested()) return;
            heading = getGyro().getHeading();
        }

    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDriveBETA(2 * 24 + 20, 0.45 ,1000, getDriveMotors());
        stopDriveMotors();
    }



    //Turn from vortex and line up with 1st beacon
    public void goToFirstBeacon() throws InterruptedException {
        statusTelemetry("Approaching...");
        encoderDriveBETA(2.0 * 24 * Math.sqrt(2) - 13, 0.7, 2, getDriveMotors());
        stopDriveMotors();
        simpleWait(10);

        double heading = gyroSensor.getHeading();
        setDriveSpeed(-1, -1);
        while(!(heading < 286) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        stopDriveMotors();
    }

    // Hit the beacon 2 times for now
    public void mashBeacon(boolean isLast) throws InterruptedException {
        statusTelemetry("Driving until 5cm");
        driveUntilStop(0.3);
        long refresh = System.currentTimeMillis() + 5000;
        //encoderDrive(2, -0.2);
        simpleWaitS(0.1);
        int blue = 0,  red = 0;

        //back up a bit before sensing colors
        encoderDriveBETA(3, -0.6, 1000, getDriveMotors());


        long endTime = System.currentTimeMillis() + 500;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(rightColorSensor.red() > rightColorSensor.blue()
                    || leftColorSensor.red() > leftColorSensor.blue())
                red ++;
            else blue ++;
            if(isStopRequested())
                return;
        }
        telemetry.addData("Beacon", "Red: " + red + " Blue: " + blue);


        if(blue > red) {
            statusTelemetry("Backing away");
            simpleWaitS(0.05);
            encoderDriveBETA(3, -0.5, 1000, getDriveMotors());
            stopDriveMotors();

            statusTelemetry("Waiting till end");
            while(refresh > System.currentTimeMillis() && opModeIsActive()) {
                if(isStopRequested()) return;
            }

            statusTelemetry("Driving until 5cm");
            driveUntilStop(0.4);
        } else {
            encoderDriveBETA(3, -0.6, 1000, getDriveMotors());
        }

        simpleWaitS(1);
        statusTelemetry("Backing away");
        simpleWaitS(0.05);
        if (isLast)
            encoderDriveBETA(8, -0.5, 1000, getDriveMotors());
        else
            encoderDriveBETA(12, -0.5, 1000, getDriveMotors());
        stopDriveMotors();
    }

    public double speedFromVoltage() {
        double voltage = (hardwareMap.voltageSensor.get("l2 l").getVoltage() + hardwareMap.voltageSensor.get("r r1").getVoltage()) / 2;
        double speed = 0.45;
        if (voltage > 13.15)
            speed = 0.4;
        else if (voltage < 12.9)
            speed = 0.47;
        return speed;
    }



}
