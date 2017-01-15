package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Main BLUE Auto")
public class MainBlueAuto extends StrykeAutonomous {

    private ModernRoboticsI2cRangeSensor wall;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        wall = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("wall"));
        calibrateGyro();

        double speed = speedFromVoltage();
        // equation derived from above
        //double speed = 7/25 * voltage + 2041/500;
        telemetry.addData("Speed", speed);
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        encoderDrive(5, 0.7, getDriveMotors());
        int heading = getGyro().getHeading();
        setDriveSpeed(0.5, 0.5);
        while((heading < 34) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }

        goToFirstBeacon(speed);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        mashBeacon(false);
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


        heading = getGyro().getHeading();
        setDriveSpeed(0.7, 0.7);
        while (heading < 200) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        encoderDrive(60 , 0.7, getDriveMotors());
        shootTwoBalls();

        manip.setPower(-0.5);

        encoderDrive(30, 0.8, getDriveMotors());
        manip.setPower(0);

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    private void goToSecondBeacon() throws InterruptedException {
        int heading = getGyro().getHeading();
        setDriveSpeed(-speedFromVoltage() - 0.2, -speedFromVoltage() - 0.2);

        while((heading > 3)){
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }
        stopDriveMotors();

        encoderDrive(24 * 3 + 15, 0.5, getDriveMotors());

        setDriveSpeed(speedFromVoltage() + 0.2, speedFromVoltage() + 0.2);
        while((heading < 70 || heading > 180)&& opModeIsActive()) {
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
//        2.5 * 24 * Math.sqrt(2) + 40
        encoderDrive(2.5 * 24 * Math.sqrt(2) + 47, speed, getDriveMotors());

        stopDriveMotors();
        simpleWaitS(0.05);
        int heading = gyroSensor.getHeading();
        setDriveSpeed(speed + 0.1, speed + 0.15);
        while((heading < 80) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
    }

    // Hit the beacon 2 times for now
    public void mashBeacon(boolean isLast) throws InterruptedException {
        statusTelemetry("Driving till stop");
        driveUntilStop(0.37);
        long refresh = System.currentTimeMillis() + 5000;
        //encoderDrive(2, -0.2);
        simpleWaitS(0.1);
        int blue = 0,  red = 0;

        // back up and sense colors
        encoderDrive(3,-0.5,getDriveMotors());


        long endTime = System.currentTimeMillis() + 500;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(beaconColor.red() > beaconColor.blue())
                red ++;
            else blue ++;
            //idle();
            if(isStopRequested())
                return;
        }
        telemetry.addData("Beacon", "Red: " + red + " Blue: " + blue);

//        if(shoot) {
//            statusTelemetry("shooting those balls");
//            encoderDrive(12, -0.5);
//            int heading = getGyro().getHeading();
//            setDriveSpeed(-0.7, -0.7);
//            while(heading > 90 && opModeIsActive()) {
//                if(isStopRequested()) return;
//                heading = getGyro().getHeading();
//            }
//            stopDriveMotors();
//            shootTwoBalls();
//            heading = getGyro().getHeading();
//            setDriveSpeed(0.7, 0.7);
//            while(heading < 273 && opModeIsActive()) {
//                if(isStopRequested()) return;
//                heading = getGyro().getHeading();
//            }
//            stopDriveMotors();
//        }

        stopDriveMotors();
        if(red > blue) {
            statusTelemetry("Backing away");
            simpleWaitS(0.05);
            encoderDrive(9 , -0.5 , getDriveMotors());
            stopDriveMotors();

            statusTelemetry("Waiting till end");
            while(refresh > System.currentTimeMillis() && opModeIsActive()) {
                if(isStopRequested()) return;
            }

            statusTelemetry("Driving till stop");
            driveUntilStop(0.5);
        }

        simpleWaitS(1);
        statusTelemetry("Backing away");
        simpleWaitS(0.05);
        if (isLast)
            encoderDrive(8, -0.5);
        else
            encoderDrive(12, -0.5);
        stopDriveMotors();
    }


    public void driveUntilStop(double speed) throws InterruptedException {
        setDriveSpeed(speed, -speed);
        double dist = wall.getDistance(DistanceUnit.CM);
        while(dist > 5 && opModeIsActive()) {
            if(isStopRequested()) return;
            dist = wall.getDistance(DistanceUnit.CM);
            telemetry.addData("dist", dist);
            telemetry.update();
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
