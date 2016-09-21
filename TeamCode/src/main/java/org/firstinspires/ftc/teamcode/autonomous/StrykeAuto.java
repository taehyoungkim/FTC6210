package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class StrykeAuto extends LinearOpMode {

    private RobotHardware robot;
    private double wheelCirc;

    public StrykeAuto() {
        this(4);
    }

    public StrykeAuto(int wheelDiam) {
        robot = new RobotHardware(hardwareMap);
        robot.initHardware();
        wheelCirc = wheelDiam * Math.PI;
    }

    public RobotHardware getRobot() {
        return robot;
    }

    public void log(String message) {
        log(this.getClass().getSimpleName(), message);
    }

    public void log(String prefix, String message) {
        DbgLog.msg(prefix, message);
    }

    public void driveDistance(double inches, double speed) {
        driveDistance(inches, speed, -speed, -1);
    }

    public void driveDistance(double inches, double speed, int timeMs) {
        driveDistance(inches, speed, -speed, timeMs);
    }

    public void driveDistance(double inches, double leftSpeed, double rightSpeed, long timeMs) {
        try {
            getRobot().resetMotorEncoders();
            getRobot().setDriveSpeed(leftSpeed, rightSpeed);
            long startTime = System.currentTimeMillis() + timeMs;
            while(getRobot().getAverageEncoderPosition(getRobot().getDriveMotors()) / 1440 * wheelCirc < inches
                    && (timeMs < 0 || System.currentTimeMillis() - startTime < timeMs))
                waitForNextHardwareCycle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            getRobot().stopDriveMotors();
        }
    }

    public void encoderTurn(double deltaDeg, double speed) {
        encoderTurn(deltaDeg, speed, -1);
    }

    public void encoderTurn(double deltaDeg, double speed, int timeMs) {
        double inchesNeeded = (18 * Math.PI) * (deltaDeg/360);
        if(inchesNeeded < 0)
            driveDistance(inchesNeeded, speed, speed, timeMs);
        else
            driveDistance(inchesNeeded, -speed, -speed, timeMs);
    }

    public void gyroTurn(int deltaDeg, double speed) {
        if(!getRobot().isGyroInitialized()){
            encoderTurn(deltaDeg, speed);
            return;
        }
        int errorTolerance = 2;
        int targetHeading = getRobot().getGyro().getHeading() + deltaDeg;
        if(targetHeading > 0)
            speed *= -1;
        while(Math.abs(getRobot().getGyro().getHeading() - targetHeading) < errorTolerance) {
            // turnSpeed = speed * (degLeft/ totalDeg)
            double turnSpeed = speed * Range.clip((getRobot().getGyro().getHeading() - targetHeading) / (deltaDeg),0.5, 1);
            getRobot().setDriveSpeed(turnSpeed, turnSpeed);
        }
        getRobot().stopDriveMotors();
    }

    public void gyroTurnTo(int targetDeg, double speed) {
        if(!getRobot().isGyroInitialized()){
            encoderTurn(targetDeg, speed);
            return;
        }
        gyroTurn(targetDeg - getRobot().getGyro().getHeading(), speed);
    }

    public void simpleWait(long ms) throws InterruptedException {
        long stopTime = System.currentTimeMillis() + ms;
        while(stopTime > System.currentTimeMillis())
            waitOneFullHardwareCycle();
    }

    public void simpleWaitS(double seconds) throws InterruptedException {
        simpleWait((long) (seconds * 1000));
    }

}
