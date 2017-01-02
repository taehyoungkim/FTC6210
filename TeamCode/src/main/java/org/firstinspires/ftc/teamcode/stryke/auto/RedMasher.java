package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Red Masher")
public class RedMasher extends StrykeAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        calibrateGyro();

        double voltage = (hardwareMap.voltageSensor.get("left drive").getVoltage() + hardwareMap.voltageSensor.get("right drive").getVoltage()) / 2;
        double speed = 0.45;
        if (voltage > 13.15)
            speed = 0.4;
        else if (voltage < 12.9)
            speed = 0.47;

        waitForStart();
        approachVortex();
        shootTwoBalls();



        goToFirstBeacon(speed);
        mashBeacon();

        goToSecondBeacon(speed);
        mashBeacon();


    }

    //Drive towards center to shoot 2 balls
    public void approachVortex() {
        driveDistance(2 * 24, 0.5);
    }

    // Shoot 2 balls into the vortex
    public void shootTwoBalls() throws InterruptedException {
        simpleWaitS(0.1);
        shootBall();
        simpleWaitS(0.5);
        ballPopper.setPosition(BALL_POPPER_POP);
        simpleWaitS(1);
        shootBall();
        ballPopper.setPosition(BALL_POPPER_IDLE);
    }

    //Turn from vortex and line up with 1st beacon
    public void goToFirstBeacon(double speed) throws InterruptedException {
        // Back away from center to get to beacon
        driveDistance(24, -0.5);
        //Angle from a 2,3,root 13 triangle
        marginTurn(360-56, speed);
        driveToLine();
        marginTurn(270, speed);
    }

    // Hit the beacon 2 times for now
    public void mashBeacon() throws InterruptedException {
        driveUntilStop(0.5);
        simpleWaitS(0.1);
        driveDistance(24, -0.5);
        simpleWaitS(5);
        driveUntilStop(0.5);
        simpleWaitS(0.1);
        driveDistance(24, -0.5);
    }

    // Line up with 2nd beacon's tape
    private void goToSecondBeacon(double speed) throws InterruptedException {
        marginTurn(0, speed);
        driveToLine();
        marginTurn(270, speed);
    }


    public void driveUntilStop(double speed) {
        int lastLeft = leftDriveBack.getCurrentPosition(),
                lastRight = rightDriveBack.getCurrentPosition();
        int deltaLeft, deltaRight;
        do {
            int currentLeft = leftDriveBack.getCurrentPosition(),
                    currentRight = rightDriveBack.getCurrentPosition();
            deltaLeft = lastLeft - currentLeft;
            deltaRight = lastRight - currentRight;
            lastLeft = currentLeft;
            lastRight = currentRight;
            setDriveSpeed(speed, -speed);
            idle();
        } while ((deltaLeft == 0 || deltaRight == 0) && opModeIsActive());
        stopDriveMotors();
    }

    public void marginTurn(int target, double speed) {
        int heading;
        do {
            heading = getGyro().getHeading();
            setDriveSpeed(speed, speed);
            idle();
        }
        while((heading > target || heading < target - 10) && opModeIsActive());
    }
}
