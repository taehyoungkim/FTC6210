package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

public class StrykeAutonomous extends StrykeOpMode {

    int wheelDiam = 6;
    private int encoderPPR = 7 * 40;
    protected final double LEFT = 0, RIGHT = 1, MIDDLE= 0.5;

    @Override
    public void initHardware() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        super.initHardware();
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void driveToLine() throws InterruptedException {
        double odsValue;
        do {
            odsValue = ods.getLightDetected();
            setDriveSpeed(0.2, -0.2);
            telemetry.addData("ODS", odsValue);
            telemetry.update();
            idle();
        }
        while(odsValue < 0.3);
        stopDriveMotors();
    }

    @Deprecated
    public void pidEncoderDrive(double inches, DcMotor... motors) throws InterruptedException {
        int pulses = (int) ((inches / (wheelDiam * Math.PI) * encoderPPR) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);

        long lastTime = System.currentTimeMillis() - 1;
        double integral = 0.0;
        double p = 0.0025; double i = 0; double d = 0.00;

        int current = Math.abs(getAverageEncoderPosition(motors));

        int target = Math.abs(pulses);
        double pastError = Math.abs(pulses);

        while(Math.abs(target - current) > 10) {

            long currentTime = System.currentTimeMillis();
            long deltaT = currentTime - lastTime;
            current = Math.abs(getAverageEncoderPosition(motors));

            int error = target - current;
            integral = integral + (error * deltaT);
            double derivative = (error - pastError)/deltaT;
            double output = p * error + (i * integral) + (d * derivative);

            output = Range.clip(output, 0.2, 1);
            if(inches < 0) output *= -1;
            if(output > 1) output = 1;
            if(output < -1) output = -1;

            setDriveSpeed(output, -output);
            telemetry.addData("Output", output + " ");
            telemetry.addData("P", p * error + " ");
            telemetry.addData("i", i * integral + " ");
            telemetry.addData("d", d * derivative + " ");
            telemetry.update();

            pastError = error;
            lastTime = currentTime;

            currentTime = System.currentTimeMillis();
            idle();
            Thread.sleep(10 - (System.currentTimeMillis() - currentTime) ,0);
        }
        setDriveSpeed(0, 0);
    }

    public void encoderDrive(double inches, double speed, DcMotor... motors) throws InterruptedException {
        if(motors.length == 0) motors = getDriveMotors();
        int pulses = (int) ((inches / (6 * Math.PI) * 280) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        setDriveSpeed(speed, -speed);
        int avg = getAverageEncoderPosition(motors);
        while(avg <= pulses) {
            avg = getAverageEncoderPosition(motors);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", avg);
            telemetry.update();
            idle();
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
        while(avg - offset<= pulses && (System.currentTimeMillis()) < endTime) {
            avg = getAverageEncoderPosition(motors);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", avg);
            telemetry.update();
            idle();
        }
        setDriveSpeed(0, 0);
    }


    /*
    ==============================================
    ================TURN METHODS==================
    ==============================================
     */

    public void turn(int deg, double pow) throws InterruptedException {
        int current = getGyro().getHeading();
        int target = current + deg;
        double sideComp = 0.15;
        if (target > 360) target -= 360;
        if(target < 0) target = 360 + target;
        boolean turnLeft = getDistance(target, current) < 0; // if we have to turn left

        while((Math.abs(getDistance(target, current)) > 1) && opModeIsActive()){
            current = getGyro().getHeading();
            boolean newTurnLeft = getDistance(target, current) < 0; // if we have to turn left
            if(newTurnLeft != turnLeft)
                return;

            if (turnLeft) {
                setLeftDriveSpeed(-pow-sideComp);
                setRightDriveSpeed(-pow);
            } else {
                setLeftDriveSpeed(pow);
                setRightDriveSpeed(pow + sideComp);
            }
            telemetry.addData("Heading", current);
            telemetry.addData("Target", target);
            telemetry.addData("Distance", getDistance(target, current));
            telemetry.update();

            idle();
        }
    }

    /*
    ==============================================
    ================HELPER METHODS================
    ==============================================
     */

    public int getDistance(int target, int current) {
        int o = target - current;
        return (((o + 180)) % 360) - 180;
    }

    public void align(double pow) throws InterruptedException {
        while(ods.getLightDetected() < 0.4 && opModeIsActive()){
            if(pow > 0) {
                setLeftDriveSpeed(pow + 0.15); // turn to the right
                setRightDriveSpeed(pow);
            } else {
                setLeftDriveSpeed(pow - 0.1); // turn to the left
                setRightDriveSpeed(pow - 0.1);
            }
            idle();
        }
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
}