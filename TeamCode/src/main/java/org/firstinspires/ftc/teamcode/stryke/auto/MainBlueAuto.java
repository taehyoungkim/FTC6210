package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Main Blue Auto")
public class MainBlueAuto extends StrykeAutonomous {


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        calibrateGyro();

        double voltage = (hardwareMap.voltageSensor.get("left drive").getVoltage() + hardwareMap.voltageSensor.get("right drive").getVoltage()) / 2;
        double speed = 0.45;
        if (voltage > 13.15)
            speed = 0.4;
        else if (voltage < 12.9)
            speed = 0.47;
        // equation derived from above
        //double speed = 7/25 * voltage + 2041/500;
        telemetry.addData("Speed", speed);
        telemetry.addData("Voltage", voltage);
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        approachVortex();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        shootTwoBalls();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        simpleWaitS(0.1);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        goToFirstBeacon(speed);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        mashBeacon();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        encoderDrive(24 * 2.5, -0.5);
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }

        statusTelemetry("Hitting cap  ");
        long endTime = System.currentTimeMillis() + 3000;
        setDriveSpeed(-0.6,-0.6);
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(isStopRequested()) return;
        }
        stopDriveMotors();

//        int heading = getGyro().getHeading();
//        setDriveSpeed(speed + 0.05, speed + 0.2);
//        while((heading < 90  || heading > 120) && opModeIsActive()) {
//            heading = getGyro().getHeading();
//            idle();
//        }
        stopDriveMotors();

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDrive(2 * 24 + 20, 0.45 , getDriveMotors());
        stopDriveMotors();
    }



    //Turn from vortex and line up with 1st beacon
    public void goToFirstBeacon(double speed) throws InterruptedException {
        statusTelemetry("Backing away...");
        // Back away from center to get to beacon
        encoderDrive(24, -0.5);

        statusTelemetry("Approaching...");
        //Angle from a 2,3,root 13 triangle
        //marginTurnTo(360 - 56, speed);
        int heading = gyroSensor.getHeading();
        setDriveSpeed(speed + 0.05, speed);
        while((heading < 36|| heading > 330) && opModeIsActive()) {
            heading = getGyro().getHeading();
            statusTelemetry("Heading " + heading);
            if(isStopRequested())
                return;
            //idle();
        }
        stopDriveMotors();

        encoderDrive(2.5 * 24 * Math.sqrt(2) + 30, 0.35, getDriveMotors());

        heading = gyroSensor.getHeading();
        setDriveSpeed(speed + 0.05, speed);
        while((heading < 86) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
            //idle();
        }
        stopDriveMotors();
    }

    // Hit the beacon 2 times for now
    public void mashBeacon() throws InterruptedException {
        statusTelemetry("Driving till stop");
        driveUntilStop(0.2);
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
        if(red > blue) {
            statusTelemetry("Backing away");
            simpleWaitS(0.1);
            encoderDrive(12, -0.5);
            stopDriveMotors();

            statusTelemetry("Waiting 5 seconds");
            simpleWaitS(5);

            statusTelemetry("Driving till stop");
            driveUntilStop(0.2);
        }
        statusTelemetry("Red: " + red + " Blue: " + blue);
        simpleWaitS(2);
        statusTelemetry("Backing away");
        simpleWaitS(0.1);
        encoderDrive(12, -0.5);
        stopDriveMotors();
    }


    public void driveUntilStop(double speed) throws InterruptedException {
        encoderDrive(24 / 4 * 3 - 3,speed, 1.3, getDriveMotors());
    }
}