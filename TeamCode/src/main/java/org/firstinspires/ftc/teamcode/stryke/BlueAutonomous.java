package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue Autonomous", group = "Auto")
public class BlueAutonomous extends StrykeAutonomous {
    private OpticalDistanceSensor ods;
    private ModernRoboticsI2cRangeSensor range;
    private ColorSensor beaconColor;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        beaconColor = hardwareMap.colorSensor.get("color");

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
        encoderDrive(55, -0.15, getDriveMotors());
        //TODO: SHOOT

        Thread.sleep(200);
        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        pidGyroTurn(45);

        telemetry.addData("Heading", getGyro().getHeading());
        telemetry.update();

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();

        driveToLine();

        Thread.sleep(100);

        // overshoot the line
        encoderDrive(5,-0.15,getDriveMotors());
//        pidGyroTurn(22);
        while(ods.getLightDetected() < 0.3){
            setDriveSpeed(0.3, 0.3);
            idle();
        }
        stopDriveMotors();

        // Drive towards the beacon
        while (range.getDistance(DistanceUnit.CM) > 10) {
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Optical", range.cmOptical());
            telemetry.update();
            setDriveSpeed(-0.15,0.15);
            idle();
        }

        //hit the becon
        encoderDrive(25, -0.3, 5000, getDriveMotors());

        telemetry.addData("Status", "Re-Calibrating the Gyro!");
        telemetry.update();
        Thread.sleep(100);
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();

        // Run into the beacon if not already correct after 5 seconds
        encoderDrive(3,0.5,getDriveMotors()); // backup
        if(beaconColor.red() > beaconColor.blue()){ // we are incorrect!
            Thread.sleep(5000);
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();

        while (beaconColor.red() > beaconColor.blue()) {
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }


        //back away
        encoderDrive(25, 0.15, getDriveMotors());
        telemetry.addData("Status", "Turning towards the second beacon");
        telemetry.update();
        turn(360-80, 0.3);
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
        while(ods.getLightDetected() < 0.3){
            setDriveSpeed(0.5, 0.5);
            idle();
        }
        stopDriveMotors();

        // Drive towards the beacon
        while (range.getDistance(DistanceUnit.CM) > 4) {
            setDriveSpeed(-0.18,0.18);
            idle();
        }
        stopDriveMotors();

        encoderDrive(2,0.5,getDriveMotors()); // backup
        if(beaconColor.red()> beaconColor.blue()){
            Thread.sleep(5000);
            encoderDrive(3,-0.5, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();
    }

    private void driveToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.4) {
            setDriveSpeed(-0.25,0.25);
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        stopDriveMotors();
    }

}
