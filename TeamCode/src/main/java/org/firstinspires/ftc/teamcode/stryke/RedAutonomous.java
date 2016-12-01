package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous", group = "Auto")
public class RedAutonomous extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();

        beaconColor.enableLed(false);
        telemetry.addData("Status", "Initializing gyro...");
        telemetry.update();
        getGyro().calibrate();
        int dots = 0;
        long nextTime = System.currentTimeMillis() + 500;
        while(getGyro().isCalibrating()){
            if(System.currentTimeMillis() > nextTime) { // Display loading animation for drivers
                nextTime = System.currentTimeMillis() + 500;
                String out = "Initializing gyro";
                for(int i = 0; i < dots % 4; i ++)
                    out += ".";
                dots ++;
                telemetry.addData("Status", out);
                telemetry.update();
            }
            idle();
        }

        telemetry.addData("Status", "Ready.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Positioning to shoot the ball");
        telemetry.update();
        encoderDrive(43, -0.15, getDriveMotors());
        //TODO: SHOOT

        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        turn(360-45, .3);

        telemetry.addData("Heading", getGyro().getHeading());
        telemetry.update();
        Thread.sleep(200);

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();

        Thread.sleep(100);
        driveToLine();

        Thread.sleep(100);

        // overshoot the line
        encoderDrive(5,-0.15,getDriveMotors());
        align(-0.3);
        stopDriveMotors();

        // Drive towards the beacon
        driveToWall(10);

        //hit the beacon
        encoderDrive(25, -0.3, 3000, getDriveMotors());
        long initialHit = System.currentTimeMillis();

        telemetry.addData("Status", "Re-Calibrating the Gyro!");
        telemetry.update();
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();

        telemetry.addData("Status", "Ready.");
        telemetry.update();
        Thread.sleep(100);

        // Run into the beacon if not already correct after 5 seconds
        encoderDrive(3,0.5,getDriveMotors()); // backup

        checkRedBeacon(initialHit);

        stopDriveMotors();

        //back away
        encoderDrive(10, 0.15, getDriveMotors());
        stopDriveMotors();

        telemetry.addData("Status", "Turning towards the second beacon");
        telemetry.update();
        turn(76, 0.3);
        stopDriveMotors();

        telemetry.addData("Status", "Driving towards the second beacon");
        telemetry.update();
        driveToLine();
        stopDriveMotors();

        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();

        //Overshoot the line a little
        encoderDrive(9, -0.18, getDriveMotors());
        stopDriveMotors();
        Thread.sleep(100);

        // Turn until we are on the left side of the tape
        align(-0.3);
        stopDriveMotors();

        // Drive towards the beacon
        driveToWall(4);
        stopDriveMotors();

        encoderDrive(2,0.5,getDriveMotors()); // backup

        checkRedBeacon();

        stopDriveMotors();

    }
}
