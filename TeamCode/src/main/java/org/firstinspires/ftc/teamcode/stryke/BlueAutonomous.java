package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Taehyoung Kim on 10/27/16.
 */

@Autonomous(name = "Blue Autonomous", group = "Auto")
public class BlueAutonomous extends StrykeAutonomous {
    private OpticalDistanceSensor ods;
    private ModernRoboticsI2cRangeSensor range;
    private ColorSensor beaconColor;

    private boolean isBeaconOneCorrect;
    private boolean isBeaconTwoCorrect;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //beaconColor = hardwareMap.colorSensor.get("beacon");
        isBeaconOneCorrect = false;
        isBeaconTwoCorrect = false;

        beaconColor.enableLed(false);
        telemetry.addData("Status", "Initializing gyro...");
        telemetry.update();
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();

        telemetry.addData("Status", "Ready.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Positioning to shoot the ball");
        telemetry.update();
        encoderDrive(37, -0.15, getDriveMotors());
        //driveDistance(20, 0.25, 3000);
        //TODO: SHOOT

        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        pidGyroTurn(45);
        sleep(1000);


        //TODO: THE ROBOT TURNS TOWARDS THE FIRST BEACON, THEN KEEPS ON TURNING INSTEAD OF DRIVING TOWARDS THE LINE...

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();
        //I did not know the alpha value for the white line, used ODS for now.
        driveToLine();


        //turn towards the beacon


        pidGyroTurn(20);


        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //align until the robot is 3 cm away from the beacon
        //align();

        while (range.getDistance(DistanceUnit.CM) > 2) {
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Optical", range.cmOptical());
            telemetry.update();
            setDriveSpeed(-0.15,0.15);
            idle();
        }
        stopDriveMotors();
        // Assume we run into the beacon when drigint towards wall, begin 5 second countdown
        long firstDebounceTime = System.currentTimeMillis() + 5 * 1000;

        telemetry.addData("Status", "Re-Calibrating the Gyro and detecting color!");
        telemetry.update();

        Thread.sleep(100);
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();
        Thread.sleep(100);

        // Run into the beacon if not already correct after 5 seconds
        solveBlueBeacon(2, firstDebounceTime);


        //encoderDrive(20, -0.15, getDriveMotors());
        //driveDistance(-10, 0.25); //run into the beacon

        //TODO: FIND THE THRESHOLD VALUE FOR BLUE
       // if(beaconColor.blue() > 8) isBeaconOneCorrect =true;

        //back away
        encoderDrive(15, 0.15, getDriveMotors());
        telemetry.addData("Status", "Turning towards the second beacon");
        telemetry.update();
        negativePidGyroTurn(-85);
        stopDriveMotors();

        telemetry.addData("Status", "Driving towards the second beacon");
        telemetry.update();
        driveToLine();
        stopDriveMotors();

        //turn towards the beacon
        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //Overshoot the line a little
        encoderDrive(9 , -0.15, getDriveMotors());
        stopDriveMotors();
        Thread.sleep(100);

        // Turn until we are on the left side of the tape
        while(ods.getLightDetected() < 0.5){
            setDriveSpeed(0.25, 0.25);
            idle();
        }
        stopDriveMotors();

        //align();

        while (range.getDistance(DistanceUnit.CM) > 2) {
            setDriveSpeed(-0.15,0.15);
            idle();
        }
        stopDriveMotors();

        solveBlueBeacon(2);

        // Get the cap ball!!!
        encoderDrive(6, 0.6, getDriveMotors());
        pidGyroTurn(45);
        encoderDrive(60, 0.6, getDriveMotors());

//        if(beaconColor.blue() > 8) isBeaconTwoCorrect = true;
//
//        if(!isBeaconOneCorrect){
//            driveDistance(10, 0.25);
//            pidGyroTurn(90);
//            driveToLine();
//            pidGyroTurn(-90);
//            align();
//            driveDistance(-10, 0.25);
//            telemetry.addData("Status", "First beacon has the wrong alliance color...");
//            telemetry.update();
//            if (!isBeaconTwoCorrect) {
//                pidGyroTurn(-90);
//                driveToLine();
//                pidGyroTurn(90);
//                align();
//                driveDistance(-10, 0.25);
//                telemetry.addData("Status", "Second beacon has the wrong alliance color...");
//                telemetry.update();
//
//            }
//        } else if(isBeaconOneCorrect && !isBeaconTwoCorrect) {
//            driveDistance(10, 0.25);
//            driveDistance(-10, 0.25);
//            telemetry.addData("Status", "Second beacon has the wrong alliance color...");
//            telemetry.update();
//        } else {
//            telemetry.addData("Status", "Pressed the right colors on the first try!");
//            telemetry.update();
//        }

        //stopDriveMotors();

    }

    private void align() throws InterruptedException {
        while (range.cmOptical() > 3) {
            if(ods.getLightDetected() >= 0.5) {
                setRightDriveSpeed(-0.2);
                setLeftDriveSpeed(0);
            } else {
                setLeftDriveSpeed(-0.2);
                setRightDriveSpeed(0);
            }
            idle();
        }
    }

    private void driveToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.5) {
            setDriveSpeed(-0.15,0.15);
            idle();
        }
    }

    private void turnToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.5) {
            setLeftDriveSpeed(-0.15);
            idle();
        }
    }

    private void solveBlueBeacon(int maxAttempts) throws InterruptedException {
        solveBlueBeacon(maxAttempts, System.currentTimeMillis());
    }

    private void solveBlueBeacon(int maxAttempts, long debounceStartTime) throws InterruptedException {
        if(beaconColor == null) return;
        while(!(beaconColor.blue() > beaconColor.red()) && maxAttempts >= 1) {
            telemetry.addData("Status", "Waiting for beacon debounce!");
            telemetry.update();
            while (System.currentTimeMillis() < debounceStartTime) idle();

            encoderDrive(5, 0.5, 1000, getDriveMotors());
            encoderDrive(5, -0.5, 1000, getDriveMotors());
            debounceStartTime = System.currentTimeMillis() + 5 * 1000;
            maxAttempts--;
        }
    }
}
