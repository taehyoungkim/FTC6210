package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Taehyoung Kim on 10/27/16.
 */

@Autonomous(name = "Red Autonomous", group = "Auto")
public class RedAutonomous extends StrykeAutonomous {
    private OpticalDistanceSensor ods;
    private ModernRoboticsI2cRangeSensor range;
    //private ColorSensor beaconColor;

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

        //beaconColor.enableLed(false);
        telemetry.addData("Status", "Initializing gyro...");
        telemetry.update();
        getGyro().calibrate();
        while(getGyro().isCalibrating()) idle();

        telemetry.addData("Status", "Ready.");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Positioning to shoot the ball");
        telemetry.update();
        encoderDrive(30, -0.25, getDriveMotors());
        //driveDistance(20, 0.25);
        //TODO: SHOOT

        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        pidGyroTurn(315);
        sleep(1000);


        //TODO: THE ROBOT TURNS TOWARDS THE FIRST BEACON, THEN KEEPS ON TURNING INSTEAD OF DRIVING TOWARDS THE LINE...

        telemetry.addData("Status", "Driving towards the first beacon");
        telemetry.update();
        //I did not know the alpha value for the white line, used ODS for now.
        driveToLine();


        //turn towards the beacon
        pidGyroTurn(340);

        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        //align until the robot is 3 cm away from the beacon
        align();

        encoderDrive(10, -0.25, getDriveMotors());
        //driveDistance(-10, 0.25); //run into the beacon

        //TODO: FIND THE THRESHOLD VALUE FOR BLUE
        // if(beaconColor.red() > 8) isBeaconOneCorrect =true;

        //back away
        driveDistance(10, 0.25);
        telemetry.addData("Status", "Turning towards the second beacon");
        pidGyroTurn(90);

        telemetry.addData("Status", "Driving towards the second beacon");
        telemetry.update();
        driveToLine();

        //turn towards the beacon
        telemetry.addData("Status", "Line found! Aligning...");
        telemetry.update();
        pidGyroTurn(-90);
        align();

        encoderDrive(10, -0.25,5000, getDriveMotors()); // drive into beacon
//
//        if(beaconColor.red() > 8) isBeaconTwoCorrect = true;
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
}
