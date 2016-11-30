package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue Autonomous", group = "Auto")
public class BlueAutonomous extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        hitter.setPosition(Servo.MIN_POSITION);
        release.setPosition(1);
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
        encoderDrive(55, 0.15, getDriveMotors());
        //TODO: SHOOT

        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        turn(45, 0.25);
        Thread.sleep(200);

        telemetry.addData("Heading", getGyro().getHeading());
        telemetry.update();

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();

        driveToLine();

        Thread.sleep(100);

        // overshoot the line
        encoderDrive(5, 0.15,getDriveMotors());
        align(0.3);
        stopDriveMotors();

        // Drive towards the beacon
        driveToWall(15);
        telemetry.addData("Distance from the wall", range.getDistance(DistanceUnit.CM));
        telemetry.update();

        // hit the beacon
        hit();

        //hit the beacon
        //encoderDrive(10, 0.3, getDriveMotors());
        //long initialHit = System.currentTimeMillis();

        telemetry.addData("Status", "Re-Calibrating the Gyro!");
        telemetry.update();
        Thread.sleep(100);
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();

        //checkBlueBeacon(initialHit);

        stopDriveMotors();

        //back away
        encoderDrive(25, -0.15, getDriveMotors());
        stopDriveMotors();
        telemetry.addData("Status", "Turning towards the second beacon");
        telemetry.update();
        turn(360-90, 0.3);
        stopDriveMotors();


        telemetry.addData("Status", "Driving towards the second beacon");
        telemetry.update();
        driveToLine();
        stopDriveMotors();

        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //Overshoot the line a little
        encoderDrive(9, 0.18, getDriveMotors());
        stopDriveMotors();
        Thread.sleep(100);

        // Turn until we are on the left side of the tape
        align(0.3);
        stopDriveMotors();

        // Drive towards the beacon
        driveToWall(4);

        stopDriveMotors();

        encoderDrive(2,-0.5,getDriveMotors()); // backup

        hit();

        //checkBlueBeacon();

        stopDriveMotors();
    }


    private void hit() throws InterruptedException {
        if (beaconColor.blue() > beaconColor.red()) {
            hitter.setPosition(Servo.MIN_POSITION);
            
        } else {
            hitter.setPosition(0.9);
        }

        Thread.sleep(500);
        encoderDrive(5,0.15, getDriveMotors()); // hit
        stopDriveMotors();
        Thread.sleep(100);
        encoderDrive(5,-0.5,getDriveMotors()); // backup
        stopDriveMotors();
        Thread.sleep(300);

         //just in case something goes wrong...
        while (beaconColor.blue() > beaconColor.red()) {
            encoderDrive(3,0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,-0.5,getDriveMotors()); // backup
            idle();
        }
    }

}
