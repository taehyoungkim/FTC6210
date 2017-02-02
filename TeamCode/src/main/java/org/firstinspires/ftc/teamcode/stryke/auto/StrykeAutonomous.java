package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

public class StrykeAutonomous extends StrykeOpMode {

    private static final int wheelDiam = 6;
    private static final int encoderPPR = 7 * 40;
    private ModernRoboticsI2cRangeSensor wall;

    @Override
    public void initHardware() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        super.initHardware();
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wall = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("wall"));

        releaseLeft.setPosition(HUGGER_LEFT_DOWN);
        releaseRight.setPosition(HUGGER_RIGHT_DOWN);
        beaconColor.enableLed(false);
        ballPopper.setPosition(BALL_POPPER_IDLE);

        telemetry.addData("Status", "Initialize done!");
        telemetry.update();
    }

    /*
    ==============================================
    ================DRIVE METHODS=================
    ==============================================
     */

    public void encoderDrive(double inches, double speed, DcMotor... motors) throws InterruptedException {
        if(motors.length == 0) motors = getDriveMotors();
        if(inches < 0) speed = -Math.abs(speed);
        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        setDriveSpeed(speed, -speed);
        int avg = getAverageEncoderPosition(motors);
        while(avg <= pulses && opModeIsActive()) {
            avg = getAverageEncoderPosition(motors);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", avg);
            telemetry.update();
            if(isStopRequested()) {
                stopDriveMotors();
                return;
            }

            //idle();
        }
        stopDriveMotors();
    }

    public void encoderDrive(double inches, double speed, double time, DcMotor... motors) throws InterruptedException {
        int offset = getAverageEncoderPosition(motors);
        int pulses = (int) ((inches / (6 * Math.PI) * 280) * 1.6);
        double endTime = System.currentTimeMillis() + time * 1000;
        resetMotorEncoders();
        //while(motors[0].isBusy()) idle();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        setDriveSpeed(speed, -speed);
        int avg = getAverageEncoderPosition(motors);
        while(avg - offset <= pulses && System.currentTimeMillis() < endTime && opModeIsActive()) {
            avg = getAverageEncoderPosition(motors);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", avg);
            telemetry.update();
            //idle();
        }
        setDriveSpeed(0, 0);
    }

    /*
    ==============================================
    ================HELPER METHODS================
    ==============================================
     */

    public void driveUntilStop(double speed) throws InterruptedException {
        setDriveSpeed(speed, -speed);
        double dist = wall.getDistance(DistanceUnit.CM);
        while(dist > 5 && opModeIsActive()) {
            if(isStopRequested()) return;
            dist = wall.getDistance(DistanceUnit.CM);
            telemetry.addData("dist", dist);
            telemetry.update();
        }
        stopDriveMotors();
    }

    public int getDistance(int target, int current) {
        int o = target - current;
        return (((o + 180)) % 360) - 180;
    }

    public void calibrateGyro() {
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
        telemetry.addData("Status", "Initialize done!");
        telemetry.update();
    }

    // Shoot 2 balls into the vortex
    public void shootTwoBalls() throws InterruptedException {
        statusTelemetry("Shooting 1st ball...");
        simpleWaitS(0.1);
        shootBall();
        statusTelemetry("Shooting 2nd ball...");
        simpleWaitS(0.5);
        ballPopper.setPosition(BALL_POPPER_POP);
        simpleWaitS(1);
        shootBall();
        ballPopper.setPosition(BALL_POPPER_IDLE);
    }
}