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
package org.firstinspires.ftc.teamcode.stryke.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stryke.AnalogDistanceFinder;
import org.firstinspires.ftc.teamcode.stryke.GamepadListener;
import org.firstinspires.ftc.teamcode.stryke.MRRangeSensor;

@TeleOp(name="Main Tele-Op", group="Linear Opmode")
public class StrykeOpMode extends LinearOpMode {

    /* Declare OpMode members. */

    public static final double HUGGER_LEFT_UP = 1;
    public static final double HUGGER_RIGHT_UP = 0.7;
    public static final double HUGGER_RIGHT_DOWN = 1;
    public static final double HUGGER_LEFT_DOWN = 0.7;
    public static final double BALL_POPPER_IDLE = 1;
    public static final double BALL_POPPER_POP = 0.2;
    public static boolean shooterReady = true;

    public static double LIFT_SPEED = 1;
    public static double SLOW_MODE_SCALE = 0.6;

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDriveBack, rightDriveBack;
    public DcMotor liftOne, liftTwo;
    public DcMotor shooter, manip;
    public GyroSensor gyroSensor;
    public OpticalDistanceSensor ods;
    //public ModernRoboticsI2cRangeSensor leftRange, rightRange;
    public ColorSensor beaconColor;
    public Servo releaseLeft, releaseRight, ballPopper;

    public Thread shootingThread;

    boolean halfSpeed = false;
    int wheelDiam = 6;
    private int encoderPPR = 7 * 40;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean debug = gamepad1.a; // Hold A before init to enter sensor testing mode

        initHardware();
        holdBallHugger();
        ballPopper.setPosition(BALL_POPPER_IDLE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        shootingThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    shootBall();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        GamepadListener gp1 = new GamepadListener(gamepad1);
        gp1.setOnReleased(GamepadListener.Button.A, new Runnable() {
            @Override
            public void run() {
                halfSpeed = !halfSpeed;
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_UP, new Runnable() {
            @Override
            public void run() {
                SLOW_MODE_SCALE = Range.clip(SLOW_MODE_SCALE + 0.05, 0.2, 0.7);
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_DOWN, new Runnable() {
            @Override
            public void run() {
                SLOW_MODE_SCALE = Range.clip(SLOW_MODE_SCALE - 0.05, 0.2, 0.7);
            }
        });

        GamepadListener gp2 = new GamepadListener(gamepad2);
        gp2.setOnPressed(GamepadListener.Button.Y, new Runnable() {
            @Override
            public void run() {
                if(!shooterReady) return;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            shooterReady = false;
                            shootBall();
                            shooterReady = true;
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }).start();
            }
        });

        stopDriveMotors();

        if(debug) {
            telemetry.addData("Status", "Calibrating Gyro");
            telemetry.update();
            gyroSensor.calibrate();
            while(gyroSensor.isCalibrating())
                idle();
        }


        telemetry.addData("Status", "Ready.");
        telemetry.update();
        waitForStart();
        runtime.reset();

        stopDriveMotors();
        while (opModeIsActive()) {

            if(debug) {
                telemetry.addData("Status", "Debug Mode");
                telemetry.addData("Gyro Heading", gyroSensor.getHeading());
                telemetry.addData("ODS", ods.getLightDetected());
                telemetry.addData("Color", beaconColor.red() > beaconColor.blue() ? "RED" : "BLUE");
            }


            gp1.update(gamepad1);
            gp2.update(gamepad2);

            // GAMEPAD 1

            // Drive controls
            if(halfSpeed){
                setDriveSpeed(scaleGamepadInput(-gamepad1.right_stick_y, -SLOW_MODE_SCALE),
                        scaleGamepadInput(-gamepad1.left_stick_y, SLOW_MODE_SCALE));
                telemetry.addData("Reversed", "Yes!");
                telemetry.addData("Speed" , SLOW_MODE_SCALE);
            }
            else{
                setDriveSpeed(scaleGamepadInput(-gamepad1.left_stick_y, 1),
                        scaleGamepadInput(-gamepad1.right_stick_y, -1));
                telemetry.addData("Reversed", "No");
            }

            // Manipulator Controls
            if(gamepad1.left_trigger > 0.1) {
                manip.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                manip.setPower(gamepad1.right_trigger);
            } else manip.setPower(0);

            if(gamepad1.b) SLOW_MODE_SCALE = 0.4;
            if(gamepad1.x) SLOW_MODE_SCALE = 0.6;


            // GAMEPAD 2
            // Lift controls
            if (gamepad2.dpad_up){
                if (runtime.seconds() > 60)
                    holdBallHugger();
                liftOne.setPower(-LIFT_SPEED);
                liftTwo.setPower(-LIFT_SPEED);
            } else if (gamepad2.dpad_down) {
                liftOne.setPower(LIFT_SPEED);
                liftTwo.setPower(LIFT_SPEED);
            } else {
                liftOne.setPower(0);
                liftTwo.setPower(0);
            }

            if (gamepad2.a)
                releaseBallHugger();

            if (gamepad2.x)
                holdBallHugger();


            if (gamepad2.dpad_left || gamepad2.b)
                ballPopper.setPosition(BALL_POPPER_POP);
            else ballPopper.setPosition(BALL_POPPER_IDLE);

            telemetry.update();
            idle();
        }
        stopDriveMotors();
    }

    public void initHardware() {
        leftDriveBack = hardwareMap.dcMotor.get("bl");
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveBack = hardwareMap.dcMotor.get("br");
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftOne = hardwareMap.dcMotor.get("one");
        liftTwo = hardwareMap.dcMotor.get("two");

        shooter = hardwareMap.dcMotor.get("shoot");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manip = hardwareMap.dcMotor.get("manip");
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        beaconColor = hardwareMap.colorSensor.get("color");

        releaseLeft = hardwareMap.servo.get("releaseL");
        releaseRight = hardwareMap.servo.get("releaseR");
        ballPopper = hardwareMap.servo.get("pop");
    }

    // **** HELPER METHODS ****
    // DcMotor Helper Methods
    public void stopDriveMotors() {
        setDriveSpeed(0, 0);
    }



    public void resetMotorEncoders(){
        telemetry.addData("Status", "reset Encoders");
        telemetry.update();
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
        setMotorSpeeds(speed, rightDriveBack);
    }

    public void setLeftDriveSpeed(double speed) {
        setMotorSpeeds(speed, leftDriveBack);
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
        return numMotors == 0 ? 0 : total/numMotors;
    }

    public DcMotor[] getDriveMotors(){
        return new DcMotor[]{leftDriveBack, rightDriveBack};
    }

    // Servo Helper Methods
    public void setServoPositionWithTime(final Servo servo, final double targetPosition, final int timeMS) {
        setServoPositionWithTime(servo, targetPosition, timeMS, 50);
    }

    public void holdBallHugger() {
        releaseLeft.setPosition(HUGGER_LEFT_DOWN);
        releaseRight.setPosition(HUGGER_RIGHT_DOWN);
    }

    public void releaseBallHugger() {
        releaseLeft.setPosition(HUGGER_LEFT_UP);
        releaseRight.setPosition(HUGGER_RIGHT_UP);
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
    public void shootBall() throws InterruptedException {
        int startPosition = shooter.getCurrentPosition();
        shooter.setPower(-0.6);
        long endTime = System.currentTimeMillis() + 3000;
        while (Math.abs(shooter.getCurrentPosition() - startPosition) < 1440 && endTime > System.currentTimeMillis() && opModeIsActive()) {
            telemetry.addData("Shooter", shooter.getCurrentPosition());
            telemetry.update();

        }
        shooter.setPower(0);
    }

    public void simpleWait(long ms) throws InterruptedException {
        long stopTime = System.currentTimeMillis() + ms;
        while(stopTime > System.currentTimeMillis() && opModeIsActive())
            idle();
    }

    public void simpleWaitS(double seconds) throws InterruptedException {
        simpleWait((long) (seconds * 1000));
    }

    public GyroSensor getGyro() {
        return gyroSensor;
    }

    public void statusTelemetry(Object data) {
        telemetry.addData("Status", data);
        telemetry.update();
    }


}