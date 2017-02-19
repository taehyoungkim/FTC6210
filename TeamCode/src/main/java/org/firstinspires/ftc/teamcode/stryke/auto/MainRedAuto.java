package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Main RED Auto") //GEN3 FIELD
public class MainRedAuto extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        calibrateGyro();

        double speed = speedFromVoltage();
        // equation derived from above
        //double speed = 7/25 * voltage + 2041/500;
        telemetry.addData("Speed", speed);
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        //getGyro().resetZAxisIntegrator();
        simpleWait(200);
        runtime.reset();

        encoderDriveBETA(15 , speedFromVoltage() , 100,getDriveMotors());

        int heading = getGyro().getHeading();
        setDriveSpeed(-0.45,-0.45);
        while((heading > 360 - 35 || heading < 90) && opModeIsActive()) {
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }
        stopDriveMotors();
        simpleWait(100);

        goToFirstBeacon(speed);
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
        setDriveSpeed(-0.5, -0.5);
        while (heading > 170) {
            heading = getGyro().getHeading();
            if(isStopRequested())
                return;
        }
        stopDriveMotors();

        encoderDriveBETA(18.5,0.5, 1000, getDriveMotors());
        shootTwoBalls();
        manipulator.setPower(-0.5);

        
        encoderDrive(30, 0.6, getDriveMotors());
        manipulator.setPower(0);

        statusTelemetry("Done with "+ (30 - runtime.seconds()) +" seconds left!");
    }

    private void goToSecondBeacon() throws InterruptedException {
        int heading = getGyro().getHeading();
        setDriveSpeed(speedFromVoltage(), speedFromVoltage());

        telemetry.addData("Heading", getGyro().getHeading());
        telemetry.update();
        while( heading < 343 && heading > 90){
            heading = getGyro().getHeading();
            if(isStopRequested()) return;
        }
        stopDriveMotors();

        encoderDriveBETA(24 * 3 - 30, 0.75,1000, getDriveMotors());

        setDriveSpeed(-speedFromVoltage(), -speedFromVoltage());
        while((heading > 283 || heading < 90) && opModeIsActive()) {
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
    public void goToFirstBeacon(double speed) throws InterruptedException {
        statusTelemetry("Approaching...");
        encoderDriveBETA(2.0 * 24 * Math.sqrt(2) - 10, speed + 1 , 2, getDriveMotors());
        stopDriveMotors();
        simpleWait(10);

        int heading = gyroSensor.getHeading();
        setDriveSpeed(-speed, -speed);
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
        driveUntilStop(0.33);
        long refresh = System.currentTimeMillis() + 5000;
        //encoderDrive(2, -0.2);
        simpleWaitS(0.1);
        int blue = 0,  red = 0;
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    encoderDrive(3,-0.5,1000,getDriveMotors());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();

        long endTime = System.currentTimeMillis() + 500;
        while(System.currentTimeMillis() < endTime && opModeIsActive()) {
            if(rightColorSensor.red() > rightColorSensor.blue())
                red ++;
            else blue ++;
            //idle();
            if(isStopRequested())
                return;
        }
        telemetry.addData("Beacon", "Red: " + red + " Blue: " + blue);


        if(blue > red) {
            statusTelemetry("Backing away");
            simpleWaitS(0.05);
            encoderDrive((12 - 3), -0.5,1000, getDriveMotors());
            stopDriveMotors();

            statusTelemetry("Waiting till end");
            while(refresh > System.currentTimeMillis() && opModeIsActive()) {
                if(isStopRequested()) return;
            }

            statusTelemetry("Driving until 5cm");
            driveUntilStop(0.5);
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
