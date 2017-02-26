package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Main BLUE Auto") //GEN 3 FIELD
public class MainBlueAuto extends StrykeAutonomous {
    private double turnSpeed = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        getGyro().calibrate();

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

        encoderDriveBETA(15, 0.7, 1000000, getDriveMotors());
        stopDriveMotors();

        double heading = getGyro().getHeading();
        setDriveSpeed(turnSpeed, turnSpeed);
        while((heading < 35 || heading > 180)  && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }

        goToFirstBeacon();
        if(isStopRequested()) {
            stopDriveMotors();
            return;
        }
        stopDriveMotors();

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
        setDriveSpeed(turnSpeed, turnSpeed);
        while (heading < 203) { // 200 kim why are you so good -- arib
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        stopDriveMotors();

        encoderDriveBETA(27 , 0.7, 1000, getDriveMotors());
        shootTwoBalls();

        manipulator.setPower(-0.5);

        encoderDriveBETA(30, 0.7, 1000, getDriveMotors());
        manipulator.setPower(0);

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    private void goToSecondBeacon() throws InterruptedException {
        statusTelemetry("Second!");
        simpleWaitS(0.5);
        double heading = getGyro().getHeading();
        setDriveSpeed(-turnSpeed, -turnSpeed);

        while((heading > 20 && heading < 270)){
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }
        stopDriveMotors();

        encoderDriveBETA(24 * 3 - 30, 0.7, 1000,getDriveMotors());

        setDriveSpeed(turnSpeed, turnSpeed);
        while((heading < 68 || heading > 180)&& opModeIsActive()) {
            if(isStopRequested()) return;
            heading = getGyro().getHeading();
        }

    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() throws InterruptedException {
        statusTelemetry("Approaching vortex");
        encoderDriveBETA(2 * 24 + 20, 0.45 , 1000, getDriveMotors());
        stopDriveMotors();
    }



    //Turn from vortex and line up with 1st beacon
    public void goToFirstBeacon() throws InterruptedException {
        statusTelemetry("Approaching...");
//        2.5 * 24 * Math.sqrt(2) + 40
        encoderDriveBETA(2.5 * 24 * Math.sqrt(2) - 27, 0.7, 1000, getDriveMotors());

        stopDriveMotors();
        simpleWaitS(0.05);
        double heading = gyroSensor.getHeading();
        setDriveSpeed(turnSpeed, turnSpeed);
        while((heading < 74) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        stopDriveMotors();
    }

    // Hit the beacon 2 times for now
    public void mashBeacon(boolean isLast) throws InterruptedException {
        statusTelemetry("Driving until 5cm");
        driveUntilStop(0.37);
        long refresh = System.currentTimeMillis() + 5000;
        simpleWaitS(0.1);
        int blue = 0,  red = 0;

        // back up and sense colors
        encoderDriveBETA(3,-0.5,1000,getDriveMotors());


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

        if(red > blue) {
            statusTelemetry("Backing away");
            simpleWaitS(0.05);
            encoderDriveBETA(3, -0.7, 1000, getDriveMotors());
            stopDriveMotors();

            statusTelemetry("Waiting till end");
            while(refresh > System.currentTimeMillis() && opModeIsActive()) {
                if(isStopRequested()) return;
            }

            statusTelemetry("Driving until 5cm");
            driveUntilStop(0.4);
        } else {
            encoderDriveBETA(3, -0.7, 1000, getDriveMotors());
        }

        simpleWaitS(1);
        statusTelemetry("Backing away");
        simpleWaitS(0.05);
        if (isLast)
            encoderDriveBETA(7, -0.7, 1000, getDriveMotors());
        else
            encoderDriveBETA(11, -0.7, 1000, getDriveMotors());
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
