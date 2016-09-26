/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Tele-Op", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class StrykeOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDriveFront, rightDriveFront, leftDriveBack, rightDriveBack;

    boolean halfSpeed = false;
    int wheelDiam = 6;
    private int encoderPPR = 7 * 40;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDriveFront = hardwareMap.dcMotor.get("fl");
        rightDriveFront = hardwareMap.dcMotor.get("fr");
        leftDriveBack = hardwareMap.dcMotor.get("bl");
        rightDriveBack = hardwareMap.dcMotor.get("br");


        GamepadListener gp1 = new GamepadListener(gamepad1);
        gp1.setOnReleased(GamepadListener.Button.A, new Runnable() {
            @Override
            public void run() {
                halfSpeed = !halfSpeed;
            }
        });

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            gp1.update(gamepad1);


            if(halfSpeed)
                setDriveSpeed(scaleGamepadInput(gamepad1.left_stick_y, -0.5),
                            scaleGamepadInput(gamepad1.right_stick_y, 0.5));
            else
                setDriveSpeed(scaleGamepadInput(gamepad1.left_stick_y, -1),
                        scaleGamepadInput(gamepad1.right_stick_y, 1));

            idle();
        }
    }

    // **** HELPER METHODS ****
    // DcMotor Helper Methods
    public void stopDriveMotors() {
        setDriveSpeed(0, 0);
    }

    public void resetMotorEncoders(){
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
    }

    public void setMotorRunMode(DcMotor.RunMode runMode, DcMotor... motors){
        for(DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setDriveSpeed(double speedLeft, double speedRight) {
        setLeftDriveSpeed(speedLeft);
        setRightDriveSpeed(speedRight);
    }

    public void setRightDriveSpeed(double speed) {
        setMotorSpeeds(speed, rightDriveBack, rightDriveFront);
    }

    public void setLeftDriveSpeed(double speed) {
        setMotorSpeeds(speed, leftDriveBack, leftDriveFront);
    }

    public void setMotorSpeeds(double speed, DcMotor... motors) {
        for (DcMotor motor : motors)
            motor.setPower(Range.clip(speed, -1, 1));
    }

    public int getAverageEncoderPosition(DcMotor... motors) {
        return getAverageEncoderPosition(true, motors);
    }

    public int getAverageEncoderPosition(boolean ignoreSign, DcMotor... motors) {
        int total = 0;
        int numMotors = motors.length;
        for(DcMotor motor : motors){
            if(motor.getCurrentPosition() == -1) {
                numMotors --;
            } else
            if(ignoreSign)
                total += Math.abs(motor.getCurrentPosition());
            else
                total += motor.getCurrentPosition();
        }
        return total/numMotors;
    }

    public DcMotor[] getDriveMotors(){
        return new DcMotor[]{leftDriveBack, leftDriveFront, rightDriveFront, rightDriveBack};
    }

    // Servo Helper Methods
    public void setServoPositionWithTime(final Servo servo, final double targetPosition, final int timeMS) {
        setServoPositionWithTime(servo, targetPosition, timeMS, 50);
    }

    // Set a servo's position with respect for time. Used for slowly setting position of a servo
    public void setServoPositionWithTime(final Servo servo, final double targetPosition, final int timeMS, final long resolution) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                double currentPosition = servo.getPosition();
                int updates = (int) (timeMS / resolution);
                double deltaPerUpdate = (targetPosition - currentPosition) / updates;
                for(int i = 0; i < updates; i++){
                    currentPosition += deltaPerUpdate;
                    // Make sure we don't go over our target position (rounding)
                    currentPosition = Range.clip(currentPosition, -Math.abs(targetPosition), Math.abs(targetPosition));
                    servo.setPosition(Range.clip(currentPosition, -1, 1));
                    try {
                        wait(resolution);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).start();
    }

    private static double[] values =
            { 0.00, 0.05, 0.09, 0.10, 0.12
                    , 0.15, 0.18, 0.24, 0.30, 0.36
                    , 0.43, 0.50, 0.60, 0.72, 0.85
                    , 1.00, 1.00
            };

    public static double scaleGamepadInput(double power, double scale){
        // Scale gamepad joystick movement in a nonlinear fashion
        if(Math.abs(power) <= 0.05)// Make sure joystick is actually being moved
            return 0;
        // clamp value between -1 and 1, the min and max values for joystick movement
        power = Range.clip(power, -1, 1);
        int index = (int) Range.clip((int) (Math.abs(power) * values.length-1), 0, values.length-1); // Clamp index between 0 and 16
        return power < 0 ? -values[index] * scale : values[index] * scale; // Return negative value if power is < 0
    }


    // AUTONOMOUS METHODS
    public void driveDistance(double inches, double speed) {
        driveDistance(inches, speed, -speed, -1);
    }

    public void driveDistance(double inches, double speed, int timeMs) {
        driveDistance(inches, speed, -speed, timeMs);
    }

    public void driveDistance(double inches, double leftSpeed, double rightSpeed, long timeMs) {
        try {
            resetMotorEncoders();
            setDriveSpeed(leftSpeed, rightSpeed);
            long startTime = System.currentTimeMillis() + timeMs;
            while(getAverageEncoderPosition(getDriveMotors()) / encoderPPR * wheelDiam < inches
                    && (timeMs < 0 || System.currentTimeMillis() - startTime < timeMs))
                idle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            stopDriveMotors();
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
        if(!isGyroInitialized()){
            encoderTurn(deltaDeg, speed);
            return;
        }
        int errorTolerance = 2;
        int targetHeading = getGyro().getHeading() + deltaDeg;
        if(targetHeading > 0)
            speed *= -1;
        while(Math.abs(getGyro().getHeading() - targetHeading) < errorTolerance) {
            // turnSpeed = speed * (degLeft/ totalDeg)
            double turnSpeed = speed * Range.clip((getGyro().getHeading() - targetHeading) / (deltaDeg),0.5, 1);
            setDriveSpeed(turnSpeed, turnSpeed);
        }
        stopDriveMotors();
    }

    public void gyroTurnTo(int targetDeg, double speed) {
//        if(!isGyroInitialized()){
//            encoderTurn(targetDeg, speed);
//            return;
//        }
//        gyroTurn(targetDeg - getGyro().getHeading(), speed);
    }

    public void simpleWait(long ms) throws InterruptedException {
        long stopTime = System.currentTimeMillis() + ms;
        while(stopTime > System.currentTimeMillis())
            waitOneFullHardwareCycle();
    }

    public void simpleWaitS(double seconds) throws InterruptedException {
        simpleWait((long) (seconds * 1000));
    }

    public GyroSensor getGyro() {
        return null;
    }

    public boolean isGyroInitialized() {
        return false;
    }
}
