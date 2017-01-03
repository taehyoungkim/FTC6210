//package org.firstinspires.ftc.teamcode.stryke.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous(name = "Red Autonomous", group = "Auto")
//public class RedAutonomous extends StrykeAutonomous {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        telemetry.setItemSeparator(" : ");
//        telemetry.addData("Status", "Initializing hardware...");
//        telemetry.update();
//        initHardware();
//
//        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        releaseLeft.setPosition(HUGGER_LEFT_DOWN);
//        releaseRight.setPosition(HUGGER_RIGHT_DOWN);
//        beaconColor.enableLed(false);
//        ballPopper.setPosition(BALL_POPPER_IDLE);
//
//        telemetry.addData("Status", "Initializing gyro...");
//        telemetry.update();
//        getGyro().calibrate();
//        int dots = 0;
//        long nextTime = System.currentTimeMillis() + 500;
//        while(getGyro().isCalibrating()){
//            if(System.currentTimeMillis() > nextTime) { // Display loading animation for drivers
//                nextTime = System.currentTimeMillis() + 500;
//                String out = "Initializing gyro";
//                for(int i = 0; i < dots % 4; i ++)
//                    out += ".";
//                dots ++;
//                telemetry.addData("Status", out);
//                telemetry.update();
//            }
//            idle();
//        }
//        telemetry.addData("Status", "Ready.");
//        telemetry.update();
//
//        waitForStart();
//
//        telemetry.addData("Status", "Driving forward..");
//        telemetry.update();
//
//        // adjust for voltage
//        double voltage = (hardwareMap.voltageSensor.get("left drive").getVoltage() + hardwareMap.voltageSensor.get("right drive").getVoltage()) / 2;
//        double speed = 0.45;
//        if (voltage > 13.15)
//            speed = 0.4;
//        else if (voltage < 12.9)
//            speed = 0.47;
//
//        encoderDrive(10, -0.4, getDriveMotors());
//        // flash the motors
//        long time = System.currentTimeMillis() + 100;
//        while (System.currentTimeMillis() < time && opModeIsActive()) {
//            setDriveSpeed(-1, 1);
//        }
//        stopDriveMotors();
//
//        turn(-46, speed);
//
//        // position to align
//        while(ods.getLightDetected() < 0.3 && opModeIsActive()) {
//            setDriveSpeed(-0.35, 0.35);
//            telemetry.addData("ODS", ods.getLightDetected());
//            telemetry.update();
//            idle();
//        }
//
//
//        stopDriveMotors();
//
//        //align with the wall
//        while (getGyro().getHeading() > 0 && opModeIsActive()) {
//            setDriveSpeed(speed, speed);
//            idle();
//        }
//
//        encoderDrive(3, -0.3, getDriveMotors());
//
//        // drive to the 1st beacon
//        driveToLine();
//        stopDriveMotors();
//
//        // align beacon presser
//
//        //TODO: BEACON ALIGNING NEEDS A LOT OF WORK!!!
//
//        encoderDrive(3, -0.3, getDriveMotors());
//        if(beaconColor.blue() > beaconColor.red()){
//            encoderDrive(1, -0.3, getDriveMotors());
//        }
//        stopDriveMotors();
//
//        // press
//        long endTime = System.currentTimeMillis() + 1000;
//        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
//            beaconRack.setPower(0.3);
//            idle();
//        }
//        beaconRack.setPower(0);
//        // retract
//        endTime = System.currentTimeMillis() + 1000;
//        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
//            beaconRack.setDirection(DcMotorSimple.Direction.REVERSE);
//            beaconRack.setPower(0.3);
//            idle();
//        }
//        beaconRack.setPower(0);
//
//
////        telemetry.addData("Status", "Leveling...");
////        telemetry.update();
////        simpleWaitS(1);
////        //Align with wall
////        double left = leftRangeSensor.distanceCm();
////        double right = rightRangeSensor.distanceCm();
////        while(left != right
////                && left != -1
////                && right != -1
////                && left != 255
////                && right != 255
////                && opModeIsActive() ) {
////
////            if(left > right)
////                setDriveSpeed(0.3,  0.3);
////            else setDriveSpeed(-0.3, -0.3);
////            left = leftRangeSensor.distanceCm();
////            right = rightRangeSensor.distanceCm();
////            telemetry.addData("Left, Right", left + " , " + right);
////            telemetry.update();
////            simpleWait(10);
////            idle();
////        }
////        stopDriveMotors();
////
////        telemetry.addData("Left Right", left + " , " + right);
////        telemetry.update();
//
//        stopDriveMotors();
//
//
//
//        // go to 2nd beacon
//        encoderDrive(4, -0.3, getDriveMotors());
//
//        while(ods.getLightDetected() < 0.3 && opModeIsActive()) {
//            setDriveSpeed(-0.35, 0.35);
//            telemetry.addData("ODS", ods.getLightDetected());
//            telemetry.update();
//            idle();
//        }
//
//        // align beacon presser
//
//        encoderDrive(3, -0.3, getDriveMotors());
//        if(beaconColor.blue() > beaconColor.red()){
//            encoderDrive(1, -0.3, getDriveMotors());
//        }
//        stopDriveMotors();
//
//        // press
//        endTime = System.currentTimeMillis() + 1000;
//        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
//            beaconRack.setPower(0.3);
//            idle();
//        }
//        beaconRack.setPower(0);
//        // retract
//        endTime = System.currentTimeMillis() + 1000;
//        while (endTime > System.currentTimeMillis() && opModeIsActive()) {
//            beaconRack.setDirection(DcMotorSimple.Direction.REVERSE);
//            beaconRack.setPower(0.3);
//            idle();
//        }
//        beaconRack.setPower(0);
//        stopDriveMotors();
//
//        turn(-42, speed);
//        encoderDrive(55, 0.3, getDriveMotors());
//
//        shootBall();
//        Thread.sleep(500);
//        ballPopper.setPosition(BALL_POPPER_POP);
//        Thread.sleep(1000);
//        shootBall();
//        ballPopper.setPosition(BALL_POPPER_IDLE);
//
//
//    }
//}
