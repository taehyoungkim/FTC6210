package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

public class StrykeAutonomous extends StrykeOpMode {

    static final int wheelDiam = 6;
    static final int encoderPPR = 7 * 40;

    @Override
    public void initHardware() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        super.initHardware();
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        huggerHolderLeft.setPosition(HUGGER_LEFT_DOWN);
        huggerHolderRight.setPosition(HUGGER_RIGHT_DOWN);
        leftColorSensor.enableLed(false);
        gate.setPosition(GATE_UP);

        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
        try {
            simpleWait(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, getDriveMotors());

        telemetry.addData("Status", "Initialize done!");
        telemetry.update();
    }

    /*
    ==============================================
    ================DRIVE METHODS=================
    ==============================================
     */

    public void encoderDrive(double inches, double speed, DcMotor... motors) throws InterruptedException {
//        if(motors.length == 0) motors = getDriveMotors();
//        if(inches < 0) speed = -Math.abs(speed);
//        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
//        resetMotorEncoders();
//        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
//        setDriveSpeed(speed, -speed);
//        int avg = getAverageEncoderPosition(motors);
//        while(avg <= pulses && opModeIsActive()) {
//            avg = getAverageEncoderPosition(motors);
//            telemetry.addData("Target", pulses);
//            telemetry.addData("Current", avg);
//            telemetry.update();
//            if(isStopRequested()) {
//                stopDriveMotors();
//                return;
//            }
//        }
//        stopDriveMotors();
        encoderDrive(inches, speed, 100000, motors);
    }

    public void encoderDrive(double inches, double speed, double timeS, DcMotor... motors) throws InterruptedException {
        double endTime = System.currentTimeMillis() + timeS * 1000;
        if(motors.length == 0) motors = getDriveMotors();
        if(inches < 0) speed = -Math.abs(speed);
        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        setDriveSpeed(speed, -speed);
        int avg = getAverageEncoderPosition(motors);
        while(avg < pulses  && opModeIsActive() && System.currentTimeMillis() < endTime) {
            avg = getAverageEncoderPosition(motors);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", avg);
            telemetry.update();
            if(isStopRequested()) {
                stopDriveMotors();
                return;
            }
        }

        stopDriveMotors();
    }

    public void encoderDriveBETA(double inches, double speed, double timeS, DcMotor... motors) {
        double endTime = System.currentTimeMillis() + timeS * 1000;
        if(motors.length == 0) motors = getDriveMotors();

//        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
//        try {
//            simpleWait(500);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, getDriveMotors());

        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
        int offset = (Math.abs(leftDrive1.getCurrentPosition()) + Math.abs(rightDrive1.getCurrentPosition()))/2;
        setDriveSpeed(speed, - speed);

        while(Math.abs((Math.abs(leftDrive1.getCurrentPosition()) + Math.abs(rightDrive1.getCurrentPosition()))/2 - offset) < pulses){
            if(isStopRequested()){
                stopDriveMotors();
                return;
            }

            telemetry.addData("Target", pulses);
            telemetry.addData("Current", Math.abs((Math.abs(leftDrive1.getCurrentPosition()) + Math.abs(rightDrive1.getCurrentPosition())))/2 - offset);
            telemetry.update();
        }
        stopDriveMotors();


    }

    /*
    ==============================================
    ================HELPER METHODS================
    ==============================================
     */

    public void driveUntilStop(double speed) throws InterruptedException {
        setDriveSpeed(speed, -speed);
        long time = System.currentTimeMillis() + (long)(1 * 1e3);
        // change distance accordingly
        while(time > System.currentTimeMillis() && opModeIsActive()) {
            if(isStopRequested()) return;
//            dist = wall.getDistance(DistanceUnit.CM);
//            telemetry.addData("dist", dist);
//            telemetry.update();
            statusTelemetry("Time: " + ((time - System.currentTimeMillis()) /1000.0));
        }
        stopDriveMotors();
    }

    public double getDistance(double target, double current) {
        double o = target - current;
        return (((o + 180)) % 360) - 180;
    }

    public void rotateTo(int heading) {
        holdRotation(0.2, heading, 100000, true);
    }

    public void holdRotation(double speed, double heading, double timeS, boolean stop) {
        long endTime = (long) (System.currentTimeMillis() + timeS * 1000);
        double error;
        double leftSpeed, rightSpeed;
        while(System.currentTimeMillis() < endTime && !isStopRequested()) {
            error = getDistance(heading, getGyro().getHeading());
            if(error <= 2) {
                leftSpeed = 0;
                rightSpeed = 0;
                if(stop)
                    endTime = 0;
            } else {
                double robotSpeed = speed;
                if(error < 0) // Turn left
                    robotSpeed = -robotSpeed;
                leftSpeed = robotSpeed + Range.clip(error * 0.005, -1, 1);
                rightSpeed = robotSpeed + Range.clip(error * 0.005, -1, 1);
            }
            setDriveSpeed(leftSpeed, rightSpeed);
        }
        stopDriveMotors();

    }

    public void driveWithLock(double inches, double speed) {
        double lock = getGyro().getHeading();
        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
        int newLeft1Count = leftDrive1.getCurrentPosition() + pulses;
        int newRight1Count = rightDrive1.getCurrentPosition() + pulses;

        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION, getDriveMotors());

        leftDrive1.setTargetPosition(newLeft1Count);
        rightDrive1.setTargetPosition(newRight1Count);
        double error;
        double correction;
        double leftSpeed, rightSpeed;
        double max;

        setDriveSpeed(-speed, speed);
        while(leftDrive1.isBusy() && rightDrive1.isBusy() && !isStopRequested()) {
            error = getDistance(lock, getGyro().getHeading());
            correction = Range.clip(error * 0.005, -1 , 1);

            double robotSpeed = speed;
            if(inches < 0) robotSpeed = -robotSpeed;
            leftSpeed = -robotSpeed + correction;
            rightSpeed = robotSpeed + correction;

            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setDriveSpeed(leftSpeed, rightSpeed);
        }
        stopDriveMotors();
    }


    // Shoot 2 balls into the vortex
    public void shootTwoBalls() throws InterruptedException {
        statusTelemetry("Shooting 1st ball...");
        simpleWaitS(0.1);
        shootBall();
        manipulator.setPower(-0.7);
        statusTelemetry("Shooting 2nd ball...");
        simpleWaitS(0.5);
        gate.setPosition(GATE_DOWN);
        simpleWaitS(0.5);
        gate.setPosition(GATE_UP);
        shootBall();
        manipulator.setPower(0);
    }
}