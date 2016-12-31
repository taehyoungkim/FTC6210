package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Blue Autonomous", group = "Auto")
public class BlueAutonomous extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setItemSeparator(" : ");
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();
        initHardware();

        releaseLeft.setPosition(HUGGER_LEFT_DOWN);
        releaseRight.setPosition(HUGGER_RIGHT_DOWN);
        beaconColor.enableLed(false);
        ballPopper.setPosition(BALL_POPPER_IDLE);

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
        encoderDrive(22, 0.3, getDriveMotors());
        shootBall();
        Thread.sleep(500);
        ballPopper.setPosition(BALL_POPPER_POP);
        Thread.sleep(1000);
        shootBall();
        ballPopper.setPosition(BALL_POPPER_IDLE);


        telemetry.addData("Status", "Turning towards the first beacon");
        telemetry.update();
        turn(35, 0.35);
        Thread.sleep(200);

        telemetry.addData("Heading", getGyro().getHeading());

        telemetry.addData("Status", "Driving..");
        telemetry.update();
        encoderDrive(40, 0.4, 8, getDriveMotors());
        // back away slightly
        //encoderDrive(1, -0.25, getDriveMotors());

        stopDriveMotors();


//        while(opModeIsActive()) {
//            telemetry.addData("left", leftRangeSensor.distanceCm());
//            telemetry.addData("right", rightRangeSensor.distanceCm());
//            telemetry.update();
//            idle();
//        }
//        telemetry.addData("Ultra" , leftRangeSensor.getUltraSonicDistance()  + " " + rightRangeSensor.getUltraSonicDistance());
//        telemetry.update();
//        Thread.sleep(3000);
//
//
//        while (leftRangeSensor.getUltraSonicDistance() > rightRangeSensor.getUltraSonicDistance()
//                && leftRangeSensor.getUltraSonicDistance() != -1 && rightRangeSensor.getUltraSonicDistance() != -1){
//            setRightDriveSpeed(-0.4);
//            Thread.sleep(10);
//            idle();
//        }

        while (getGyro().getHeading() > 0 && opModeIsActive()) {
            setDriveSpeed(-0.4, -0.4);
            idle();
        }


        // drive to the 2nd beacon
        driveToLine();

        telemetry.addData("Status", "Leveling...");
        telemetry.update();
        simpleWaitS(1);
        // align with wall
//        double left = leftRangeSensor.distanceCm();
//        double right = rightRangeSensor.distanceCm();
//        double prevLeft = left;
//        double prevRight = right;
//        while(left != right && opModeIsActive() ) {
//            left = leftRangeSensor.distanceCm();
//            right = rightRangeSensor.distanceCm();
//
//            // eliminate error values
//            if (left == -1 || left == 255)
//                left = prevLeft;
//            else prevLeft = left;
//            if (right == -1 || right == 255)
//                right = prevRight;
//            else prevRight = right;
//
//            if(left < right)
//                setDriveSpeed(-0.35,  -0.35);
//            else setDriveSpeed(0.35, 0.35);
//
//            telemetry.addData("Left, Right", left + " , " + right);
//            telemetry.update();
//            //simpleWait(10);
//            idle();
//        }
        stopDriveMotors();



//        telemetry.addData("Left Right", left + " , " + right);
//        telemetry.update();
//        simpleWaitS(4);

        stopDriveMotors();

        // align beacon presser
        if(beaconColor.red() > beaconColor.blue()){
            encoderDrive(2, -0.4, getDriveMotors());
        }
        stopDriveMotors();

        // press
        long endTime = System.currentTimeMillis() + 1000;
        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
            beaconRack.setPower(0.3);
            idle();
        }
        beaconRack.setPower(0);
        // retract
        endTime = System.currentTimeMillis() + 1000;
        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
            beaconRack.setDirection(DcMotorSimple.Direction.REVERSE);
            beaconRack.setPower(0.3);
            idle();
        }
        beaconRack.setPower(0);

        // go back for the first beacon
        while(ods.getLightDetected() < 0.3 && opModeIsActive()) {
            setDriveSpeed(-0.4, 0.4);
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        stopDriveMotors();


        // align beacon presser
        if(beaconColor.red() > beaconColor.blue()){
            encoderDrive(2, -0.4, getDriveMotors());
        }
        stopDriveMotors();

        //press
        endTime = System.currentTimeMillis() + 1000;
        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
            beaconRack.setDirection(DcMotorSimple.Direction.FORWARD);
            beaconRack.setPower(0.3);
            idle();
        }
        beaconRack.setPower(0);
        stopDriveMotors();

    }


}
