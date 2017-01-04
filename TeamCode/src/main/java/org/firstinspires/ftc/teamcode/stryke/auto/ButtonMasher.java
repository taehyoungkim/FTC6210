package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Button Masher")
public class ButtonMasher extends StrykeAutonomous {

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
        shootTwoBalls();

        simpleWaitS(0.1);

        goToFirstBeacon(speed);
        mashBeacon();

        encoderDrive(24 * 2.5, -0.5);
        int heading = getGyro().getHeading();
        setDriveSpeed(speed + 0.05, speed + 0.2);
        while((heading < 90  || heading > 120) && opModeIsActive()) {
            heading = getGyro().getHeading();
            idle();
        }
        stopDriveMotors();

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDrive(2 * 24 + 20, 0.45 , getDriveMotors());
        stopDriveMotors();
    }

    // Shoot 2 balls into the vortex
    public void shootTwoBalls() throws InterruptedException {
        statusTelemetry("Shooting 1st ball...");
        simpleWaitS(0.1);
        shootBall();
        statusTelemetry("Shooting 2nd ball...");
        simpleWaitS(0.5);
        ballPopper.setPosition(BALL_POPPER_POP);
        simpleWaitS(1);
        shootBall();
        ballPopper.setPosition(BALL_POPPER_IDLE);
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
        setDriveSpeed(-speed, -speed);
        while(!(heading < 360-36 && heading > 180) && opModeIsActive()) {
            heading = getGyro().getHeading();

            idle();
        }
        stopDriveMotors();

        encoderDrive(2.5 * 24 * Math.sqrt(2) + 18, 0.4);

        heading = gyroSensor.getHeading();
        setDriveSpeed(-speed, -speed);
        while(!(heading < 270) && opModeIsActive()) {
            heading = getGyro().getHeading();
            idle();
        }
    }

    // Hit the beacon 2 times for now
    public void mashBeacon() throws InterruptedException {
        statusTelemetry("Driving till stop");
        driveUntilStop(0.5);
        //encoderDrive(2, -0.2);
        simpleWaitS(0.5);
        int blue = 0,  red = 0;
        long endTime = System.currentTimeMillis() + 2000;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(beaconColor.red() > beaconColor.blue())
                red ++;
            else blue ++;
            idle();
        }
        if(blue > red) {
            statusTelemetry("Backing away");
            simpleWaitS(0.1);
            encoderDrive(12, -0.5);
            stopDriveMotors();

            statusTelemetry("Waiting 5 seconds");
            simpleWaitS(5);

            statusTelemetry("Driving till stop");
            driveUntilStop(0.5);
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

    public void statusTelemetry(Object data) {
        telemetry.addData("Status", data);
        telemetry.update();
    }
}
