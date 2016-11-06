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
        beaconColor = hardwareMap.colorSensor.get("color");
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

        telemetry.addData("Heading", getGyro().getHeading());
        telemetry.update();



        //TODO: THE ROBOT TURNS TOWARDS THE FIRST BEACON, THEN KEEPS ON TURNING INSTEAD OF DRIVING TOWARDS THE LINE...

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();
        //I did not know the alpha value for the white line, used ODS for now.
        driveToLine();

        Thread.sleep(100);

        // overshoot the line
        encoderDrive(4,-0.15,100,getDriveMotors());
        // Turn until we are on the left side of the tape
        while(ods.getLightDetected() < 0.5){
            setDriveSpeed(0.18, 0.18);
            idle();
        }
        while(ods.getLightDetected() > 0.5){
            setDriveSpeed(0.18, 0.18);
            idle();
        }
        stopDriveMotors();

        // Drive towards the beacon
        while (range.getDistance(DistanceUnit.CM) > 4) {
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Optical", range.cmOptical());
            telemetry.update();
            setDriveSpeed(-0.15,0.15);
            idle();
        }
        stopDriveMotors();

        // Assume we run into the beacon when right towards wall, begin 5 second countdown
        long firstDebounceTime = System.currentTimeMillis() + 5 * 1000;

        telemetry.addData("Status", "Re-Calibrating the Gyro!");
        telemetry.update();
        Thread.sleep(100);
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();
        Thread.sleep(100);

        // Run into the beacon if not already correct after 5 seconds
        encoderDrive(2,0.5,getDriveMotors()); // backup
        if(beaconColor.red() > beaconColor.blue()){ // we are incorrect!
            while(firstDebounceTime > System.currentTimeMillis()) idle();
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();

        //TODO: TURN TOWARDS 2nd BEACON

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

        //TODO Seconds Beacon
        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //Overshoot the line a little
        encoderDrive(7, -0.18, getDriveMotors());
        stopDriveMotors();
        Thread.sleep(100);

        // Turn until we are on the left side of the tape
        while(ods.getLightDetected() < 0.5){
            setDriveSpeed(0.2, 0.2);
            idle();
        }
        stopDriveMotors();

        // Drive towards the beacon
        while (range.getDistance(DistanceUnit.CM) > 4) {
            setDriveSpeed(-0.18,0.18);
            idle();
        }
        stopDriveMotors();

        long lastDebounceTime = System.currentTimeMillis() + 5 * 1000;

        encoderDrive(2,0.5,getDriveMotors()); // backup
        if(beaconColor.red()> beaconColor.blue()){
            while(lastDebounceTime > System.currentTimeMillis()) idle();
            encoderDrive(3,-0.5, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();

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
        stopDriveMotors();
    }

    private void driveToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.4) {
            setDriveSpeed(-0.3,0.3);
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        stopDriveMotors();
    }

    private void turnToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.5) {
            setLeftDriveSpeed(-0.15);
            idle();
        }
        stopDriveMotors();
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
            if(beaconColor.blue() > beaconColor.red()) return;
            debounceStartTime = System.currentTimeMillis() + 5 * 1000;
            maxAttempts--;
        }
    }
}
