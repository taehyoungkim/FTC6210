package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "Main Red Auto")
public class MainRedAuto extends StrykeAutonomous {

    private OpticalDistanceSensor wall;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        wall = hardwareMap.opticalDistanceSensor.get("wall");
        calibrateGyro();

        double speed = speedFromVoltage();
        // equation derived from above
        //double speed = 7/25 * voltage + 2041/500;
        telemetry.addData("Speed", speed);
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        encoderDrive(5,0.7, getDriveMotors());
        int heading = getGyro().getHeading();
        setDriveSpeed(0.5,0.5);
        while((heading > 360 - 34 || heading < 90) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }

        goToFirstBeacon(speed);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        mashBeacon(true);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        goToSecondBeacon();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }
        mashBeacon(false);


        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    private void goToSecondBeacon() throws InterruptedException {
        int heading = getGyro().getHeading();
        setDriveSpeed(-speedFromVoltage(), -speedFromVoltage());
        while( heading < 3 && heading > 180){
            if(isStopRequested()) return;
        }
        stopDriveMotors();

        encoderDrive(24 * 3, 0.5, getDriveMotors());

        setDriveSpeed(speedFromVoltage(), speedFromVoltage());
        while((heading > 276 || heading < 90) && opModeIsActive()) {
            if(isStopRequested()) return;
            heading = getGyro().getHeading();
        }

    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDrive(2 * 24 + 20, 0.45 , getDriveMotors());
        stopDriveMotors();
    }



    //Turn from vortex and line up with 1st beacon
    public void goToFirstBeacon(double speed) throws InterruptedException {
        statusTelemetry("Approaching...");
        encoderDrive(2.5 * 24 * Math.sqrt(2) + 40, 0.35, getDriveMotors());

        int heading = gyroSensor.getHeading();
        setDriveSpeed(-speed - 0.05, -speed);
        while(!(heading < 276) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
    }

    // Hit the beacon 2 times for now
    public void mashBeacon(boolean shoot) throws InterruptedException {
        statusTelemetry("Driving till stop");
        driveUntilStop(0.5);
        long refresh = System.currentTimeMillis() + 5000;
        //encoderDrive(2, -0.2);
        simpleWaitS(0.5);
        int blue = 0,  red = 0;
        long endTime = System.currentTimeMillis() + 2000;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(beaconColor.red() > beaconColor.blue())
                red ++;
            else blue ++;
            //idle();
            if(isStopRequested())
                return;
        }
        telemetry.addData("Beacon", "Red: " + red + " Blue: " + blue);

        if(shoot) {
            statusTelemetry("shooting those balls");
            encoderDrive(12, -0.5);
            int heading = getGyro().getHeading();
            setDriveSpeed(0.4, 0.4);
            while(heading > 90 && opModeIsActive()) {
                if(isStopRequested()) return;
                heading = getGyro().getHeading();
            }
            stopDriveMotors();
            shootTwoBalls();
            heading = getGyro().getHeading();
            setDriveSpeed(-0.4, -0.4);
            while(heading < 276 && opModeIsActive()) {
                if(isStopRequested()) return;
                heading = getGyro().getHeading();
            }
            stopDriveMotors();
        }

        if(blue > red) {
            statusTelemetry("Backing away");
            simpleWaitS(0.1);
            encoderDrive(12, shoot ? 0.5 : -0.5);
            stopDriveMotors();

            statusTelemetry("Waiting till end");
            while(refresh > System.currentTimeMillis() && opModeIsActive()) {
                if(isStopRequested()) return;
            }

            statusTelemetry("Driving till stop");
            driveUntilStop(0.5);
        }

        simpleWaitS(2);
        statusTelemetry("Backing away");
        simpleWaitS(0.1);
        encoderDrive(12, -0.5);
        stopDriveMotors();
    }


    public void driveUntilStop(double speed) throws InterruptedException {
        setDriveSpeed(speed, -speed);
        while(wall.getLightDetected() < 500 && opModeIsActive()) {
            if(isStopRequested()) return;
        }
        stopDriveMotors();
    }

    public double speedFromVoltage() {
        double voltage = (hardwareMap.voltageSensor.get("left drive").getVoltage() + hardwareMap.voltageSensor.get("right drive").getVoltage()) / 2;
        double speed = 0.45;
        if (voltage > 13.15)
            speed = 0.4;
        else if (voltage < 12.9)
            speed = 0.47;
        return speed;
    }


}
