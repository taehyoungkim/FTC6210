package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Red Autonomous", group = "Auto")
public class RedAutonomous extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        hitter.setPosition(MIDDLE);
        releaseLeft.setPosition(1);
        releaseRight.setPosition(0.7);
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
        encoderDrive(10, 0.15, getDriveMotors());
        //TODO: SHOOT

        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        turn(-20, 0.23);
        Thread.sleep(200);

        telemetry.addData("Heading", getGyro().getHeading());

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();

        while(ods.getLightDetected() < 0.3) {
            setDriveSpeed(0.15, -0.2);
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        stopDriveMotors();

        Thread.sleep(100);

        // overshoot the line
        encoderDrive(11, 0.15, getDriveMotors());
        align(-0.25);


//        while (leftRange.cmUltrasonic() > rightRange.cmUltrasonic()){
//            setDriveSpeed(0.1,0.1);
//        }
//        while(ods.getLightDetected() > 0.3) {
//            setDriveSpeed(0.15, 0.1);
//            idle();
//        }
//
//        while(ods.getLightDetected() < 0.3) {
//            setDriveSpeed(-0.15, -0.1);
//            idle();
//        }

        stopDriveMotors();

        // Drive towards the beacon
        resetMotorEncoders();
        encoderDrive(24,0.2,2.5,getDriveMotors());
        stopDriveMotors();
        // hit the beacon
        hit();

        //hit the beacon
        //encoderDrive(10, 0.3, getDriveMotors());
        //long initialHit = System.currentTimeMillis();
//        telemetry.addData("Status", "Re-Calibrating the Gyro!");
//        telemetry.update();
//        getGyro().resetZAxisIntegrator();
//        Thread.sleep(1000 * 3);
//        while(getGyro().isCalibrating())
//            idle();


        //checkBlueBeacon(initialHit);

        stopDriveMotors();

        //back away
        encoderDrive(8, -0.1, getDriveMotors());

        hitter.setPosition(MIDDLE);
        stopDriveMotors();
        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the second beacon");
        telemetry.update();
        turn(80, 0.2);
        stopDriveMotors();



        telemetry.addData("Status", "Driving towards the second beacon");
        telemetry.update();
        driveToLine();
        Thread.sleep(100);

        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //Overshoot the line a little
        encoderDrive(9, 0.13, getDriveMotors());
        stopDriveMotors();

        // Turn until we are on the left side of the tape
        align(-0.25);
        stopDriveMotors();


        // Drive towards the beacon
        encoderDrive(24,0.2,2.5,getDriveMotors());

        stopDriveMotors();

        hit();

        //checkBlueBeacon();

        stopDriveMotors();
    }


    private void hit() throws InterruptedException {
        Thread.sleep(50); // Stop for sensor
        boolean left = beaconColor.red() > beaconColor.blue();
        Thread.sleep(50);
        telemetry.addData("Status", "Left = " + left);
        telemetry.update();
        //backup
        encoderDrive(4,-0.2,1,getDriveMotors());
        Thread.sleep(500);
        //Align servo
        if (left) {
            hitter.setPosition(LEFT);

        } else {
            hitter.setPosition(RIGHT);
        }

        Thread.sleep(500);
        encoderDrive(10 ,0.2, 3, getDriveMotors()); // hit
        stopDriveMotors();
        Thread.sleep(300);
        encoderDrive(3,-0.5, 1, getDriveMotors()); // backup again
        stopDriveMotors();

//         //just in case something goes wrong...
//        while (beaconColor.red() > beaconColor.blue()) {
//            encoderDrive(3,-0.5,getDriveMotors()); // backup
//            stopDriveMotors();
//            Thread.sleep(100);
//            encoderDrive(3,0.15, getDriveMotors()); // hit again
//            stopDriveMotors();
//            idle();
//        }
    }
}
