package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StrykeAutonomous extends StrykeOpMode {

    int wheelDiam = 6;
    private int encoderPPR = 7 * 40;

    /*
    ==============================================
    ================DRIVE METHODS=================
    ==============================================
     */

    public void driveToLine() throws InterruptedException {
        while(ods.getLightDetected() < 0.4) {
            setDriveSpeed(-0.25,0.25);
            telemetry.addData("ODS", ods.getLightDetected());
            telemetry.update();
            idle();
        }
        stopDriveMotors();
    }

    public void driveToWall(double cm) throws InterruptedException {
        while (range.getDistance(DistanceUnit.CM) > cm) {
            setDriveSpeed(-0.18, 0.18);
            idle();
        }
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
        int pulses = (int) ((inches / (6 * Math.PI) * 280) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        while(getAverageEncoderPosition(motors) <= pulses) {
            setDriveSpeed(speed, -speed);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", getAverageEncoderPosition(motors));
            telemetry.update();
            idle();
        }
        setDriveSpeed(0, 0);
    }

    public void encoderDrive(double inches, double speed, int time, DcMotor... motors) throws InterruptedException {
        int pulses = (int) ((inches / (6 * Math.PI) * 280) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        while(getAverageEncoderPosition(motors) <= pulses || (System.currentTimeMillis() * 1000) < time) {
            setDriveSpeed(speed, -speed);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", getAverageEncoderPosition(motors));
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
        if (target > 360) target -= 360;
        if(target < 0) target = 360 + target;
        boolean turnLeft = getDistance(target, current) < 0; // if we have to turn left

        while(Math.abs(getDistance(target, current)) > 2){
            current = getGyro().getHeading();
            if (turnLeft) {
                setDriveSpeed(-pow, -pow);
            } else {
                setDriveSpeed(pow, pow);
            }
            telemetry.addData("Heading", current);

            idle();
        }
    }

    // RIP
    @Deprecated
    public void pidGyroTurn(int deltaDeg) throws InterruptedException {
        long lastTime = System.currentTimeMillis() - 1;
        double integral = 0.0;
        double p = 0.0055; double i = 0.0005; double d = 0.000;

        int current = getGyro().getHeading();
        //if(current > 180) current = current - 360;
        int initial = current;
        int target = initial + deltaDeg;

        //if(target > 180) target = target - 360;


        double pastError = deltaDeg;
        int error = target - current;

        int speedScale = 1;
        if(deltaDeg < 0) // Turning left
            speedScale = -1;

        while(Math.abs(getDistance(target, current)) > 1) { // if error is greater than 2 deg
            long currentTime = System.currentTimeMillis();
            long deltaT = currentTime - lastTime;
            current = getGyro().getHeading();

            //if(current > 180) current = current - 360;

            error = getDistance(target, current);

            integral = (integral) + (0.5  * deltaT);
            double derivative = (error - pastError)/deltaT;
            double output = p * error + i * integral + d * derivative;

            if(output < 0) output = Range.clip(output, -0.3, -1);
            else if (output > 0) output = Range.clip(output, 0.3, 1);

            telemetry.addData("Output", output + " ");
            telemetry.addData("p", p * error + " ");
            telemetry.addData("i", i * integral + " ");
            telemetry.addData("d", d * derivative + " ");
            telemetry.addData("Error", error);
            telemetry.addData("Current", current);
            telemetry.addData("Actual Delta", initial - current );
            telemetry.update();
            setDriveSpeed(output, output);

            pastError = error;
            lastTime = currentTime;

            currentTime = System.currentTimeMillis();
            idle();
            Thread.sleep(Range.clip(10 - (int) (System.currentTimeMillis() - currentTime), 0, 10) ,0);
        }
        stopDriveMotors();
    }

    @Deprecated
    public void encoderTurn(double deg, double speed, DcMotor... motors) throws InterruptedException {
        //                    Arc%     * Circumference  / Dist. per rotation * ppr * percent error
        int pulses = (int) ((((deg/360) * (18 * Math.PI) / (6 * Math.PI) * 280) * 1.6) * 1.55);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        while(getAverageEncoderPosition(motors) <= pulses) {
            setDriveSpeed(speed, speed);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", getAverageEncoderPosition(motors));
            telemetry.update();
            idle();
        }

        setDriveSpeed(0, 0);

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
        while(ods.getLightDetected() < 0.3){
            setDriveSpeed(pow, pow);
            idle();
        }
    }

    public void checkBlueBeacon() throws InterruptedException {
        checkBlueBeacon(System.currentTimeMillis());
    }

    public void checkRedBeacon() throws InterruptedException {
        checkRedBeacon(System.currentTimeMillis());
    }

    public void checkBlueBeacon(long initialHit) throws InterruptedException {
        if(beaconColor.red() > beaconColor.blue()){ // we are incorrect!
            Thread.sleep((long) (5000 - Range.clip(System.currentTimeMillis() - initialHit, 0, 5000)));
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();
        Thread.sleep(300);  // Allow for beacon to flash
        while (beaconColor.red() > beaconColor.blue()) {
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
            idle();
        }
    }

    public void checkRedBeacon(long initialHit) throws InterruptedException {
        if (beaconColor.blue() > beaconColor.red()){ // we are incorrect!
            Thread.sleep((long) (5000 - Range.clip(System.currentTimeMillis() - initialHit, 0, 5000)));
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
        }
        stopDriveMotors();
        Thread.sleep(300);  // Allow for beacon to flash
        while (beaconColor.blue() > beaconColor.red()) {
            encoderDrive(3,-0.15, getDriveMotors()); // hit again
            stopDriveMotors();
            Thread.sleep(100);
            encoderDrive(3,0.5,getDriveMotors()); // backup
            idle();
        }
    }
}