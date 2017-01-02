package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Button Masher")
public class ButtonMasher extends StrykeAutonomous {

    boolean red;

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

        red = !gamepad1.x;
        telemetry.addData("Selected", red ? "RED" : "BLUE");
        telemetry.addData("Speed", speed);
        telemetry.addData("Voltage", voltage);
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        approachVortex();
        shootTwoBalls();

        simpleWaitS(0.1);

        goToFirstBeacon(speed);
        mashBeacon();

        simpleWaitS(0.1);

        goToSecondBeacon(speed);
        mashBeacon();

        statusTelemetry("Done in " + runtime.seconds() + " seconds!");

    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDrive(2 * 24 + 20, 0.5 , getDriveMotors());
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
        encoderDrive(24 + 10, -0.5);

        statusTelemetry("Approaching...");
        //Angle from a 2,3,root 13 triangle
        //marginTurnTo(360 - 56, speed);
        int heading = gyroSensor.getHeading();
        while(!(heading < 360-38 && heading > 180) && opModeIsActive()) {
            heading = getGyro().getHeading();
            setDriveSpeed(-speed, -speed);
            idle();
        }
        stopDriveMotors();

        driveToLine();
        while(!(heading < 270) && opModeIsActive()) {
            heading = getGyro().getHeading();
            setDriveSpeed(-speed, -speed);
            idle();
        }
    }

    // Hit the beacon 2 times for now
    public void mashBeacon() throws InterruptedException {
        statusTelemetry("Driving till stop");
        driveUntilStop(0.5);

        statusTelemetry("Backing away");
        simpleWaitS(0.1);
        encoderDrive(24, -0.5);
        stopDriveMotors();

        statusTelemetry("Waiting 5 seconds");
        simpleWaitS(5);


        statusTelemetry("Driving till stop");
        driveUntilStop(0.5);

        statusTelemetry("Backing away");
        simpleWaitS(0.1);
        encoderDrive(24, -0.5);
        stopDriveMotors();
    }

    // Line up with 2nd beacon's tape
    private void goToSecondBeacon(double speed) throws InterruptedException {
        statusTelemetry("Approaching...");
        int heading = getGyro().getHeading();
        while(!(heading > 0 && heading < 90) && opModeIsActive()) {
            heading = getGyro().getHeading();
            setDriveSpeed(speed, speed);
            idle();
        }
        driveToLine();
        heading = getGyro().getHeading();
        while(!(heading < 270 && heading > 200) && opModeIsActive()) {
            heading = getGyro().getHeading();
            setDriveSpeed(-speed, -speed);
            idle();
        }
    }


    public void driveUntilStop(double speed) {
        int lastLeft = leftDriveBack.getCurrentPosition(),
                lastRight = rightDriveBack.getCurrentPosition();
        int deltaLeft, deltaRight;
        do {
            int currentLeft = leftDriveBack.getCurrentPosition(),
                    currentRight = rightDriveBack.getCurrentPosition();
            deltaLeft = lastLeft - currentLeft;
            deltaRight = lastRight - currentRight;
            lastLeft = currentLeft;
            lastRight = currentRight;
            setDriveSpeed(-speed, speed);
            idle();
        } while ((deltaLeft < 2 || deltaRight < 2) && opModeIsActive());
        stopDriveMotors();
    }

    public void marginTurnTo(int target, double speed) {
        if (!red && target != 0 && target != 180)
            target = Math.abs(360 - target);

        int heading = getGyro().getHeading();
        double error = getDistance(target, heading);
        boolean left = error < 0;
        if (left) speed *= -1;
        do {
            heading = getGyro().getHeading();
            setDriveSpeed(speed, speed);
            error = getDistance(target, heading);
            idle();
        }
        while (Math.abs(error) < 4 && opModeIsActive());
        stopDriveMotors();
    }

    public void statusTelemetry(Object data) {
        telemetry.addData("Status", data);
        telemetry.update();
    }

}
