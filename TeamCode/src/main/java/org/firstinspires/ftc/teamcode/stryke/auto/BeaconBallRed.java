package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Ball Autonomous", group = "Auto")
public class BeaconBallRed extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();

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

        stopDriveMotors();

        // Drive towards the beacon
        resetMotorEncoders();
        encoderDrive(24,0.2,2.5,getDriveMotors());
        stopDriveMotors();

        // hit the beacon
        hit();
        resetMotorEncoders();
        while(rightDriveBack.isBusy()) idle();
        encoderDrive(24*2 + 5, -0.2,5,getDriveMotors());
        stopDriveMotors();
        turn(90,0.3);
    }


    private void hit() throws InterruptedException {
        Thread.sleep(50); // Stop for sensor
        boolean left = beaconColor.blue() < beaconColor.red();
        Thread.sleep(50);
        telemetry.addData("Status", "Left = " + left);
        telemetry.update();
        //backup
        encoderDrive(4,-0.2,1,getDriveMotors());
        Thread.sleep(500);
        //Align servo
        if (left) {


        } else {

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

