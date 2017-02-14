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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.stryke.BreakBeamSensor;
import org.firstinspires.ftc.teamcode.stryke.GamepadListener;

@TeleOp(name="Main Tele-Op", group="Linear Opmode")
public class StrykeOpMode extends LinearOpMode {

    /* Declare OpMode members. */

    public static final double HUGGER_LEFT_UP = 1;
    public static final double HUGGER_RIGHT_UP = 0;
    public static final double HUGGER_RIGHT_DOWN = 0.2;
    public static final double HUGGER_LEFT_DOWN = 0.7;
    public static final double GATE_UP = 0;
    public static final double GATE_DOWN = 0.5;
    public static boolean shooterReady = true, rapidReady = true;
    private double velocityLeft = 0.0, lastSpeedLeft = 0.0, velocityRight = 0.0, lastSpeedRight = 0.0;
    private long lastTime;

    public static double LIFT_SPEED = 1;
    public static double SHOOTER_SPEED = 0.7;
    public static double SLOW_MODE_SCALE = 0.6;

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;
    public DcMotor lift1, lift2;
    public DcMotor shooter, manipulator;
    public GyroSensor gyroSensor;
    //public ModernRoboticsI2cRangeSensor leftRange, rightRange;
    public ColorSensor leftColorSensor, rightColorSensor;
    public Servo huggerHolderLeft, huggerHolderRight, gate;
    public BreakBeamSensor beam;

    public Thread shootingThread, manipulatorThread;

    boolean halfSpeed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        final boolean debug = gamepad1.a; // Hold A before init to enter sensor testing mode

        initHardware();
        holdBallHugger();
        gate.setPosition(GATE_DOWN);
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

        gp1.setOnPressed(GamepadListener.Button.Y, new Runnable() {
            @Override
            public void run() {
                if(debug)
                    gyroSensor.resetZAxisIntegrator();
            }
        });

        gp1.setOnPressed(GamepadListener.Button.B, new Runnable() {
            @Override
            public void run() {
                if(debug)
                    beam.toggle();
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
                            if (gamepad2.y) {

                            }

                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }).start();
            }
        });


        stopDriveMotors();

        if(debug) {
            calibrateGyro();
        }


        telemetry.addData("Status", "Ready.");
        telemetry.update();
        waitForStart();
        runtime.reset();
        if(debug)
            gyroSensor.resetZAxisIntegrator();

        stopDriveMotors();
        lastTime = System.currentTimeMillis();
        while (opModeIsActive()) {

            if(debug) {
                telemetry.addData("Status", "Debug Mode");
                telemetry.addData("Gyro Heading", gyroSensor.getHeading() + "° from origin.");
                telemetry.addData("Left Color", leftColorSensor.red() > leftColorSensor.blue() ? "RED" : "BLUE");
                telemetry.addData("Right Color", rightColorSensor.red() > rightColorSensor.blue() ? "RED" : "BLUE");
                //telemetry.addData("Beam", beam.isBroken() ? "BROKEN" : "OPEN");
            }


            gp1.update(gamepad1);
            gp2.update(gamepad2);

            // GAMEPAD 1

            double targetLeft = scaleGamepadInput(gamepad1.left_stick_y, 1);
            double targetRight = scaleGamepadInput(gamepad1.right_stick_y, 1);
            double errorLeft = targetLeft - velocityLeft;
            double errorRight = targetRight - velocityRight;

            long currentTime = System.currentTimeMillis();
            long delta = currentTime - lastTime;
            lastTime = currentTime;

//            if (targetLeft == 0)
//                velocityLeft = Range.clip(velocityLeft + (errorLeft * 5 * delta/1000), -1, 1);
//            else if (targetRight == 0)
//                velocityRight = Range.clip(velocityRight + (errorRight * 5 * delta/1000), -1, 1);
//            else {
//                velocityRight = gamepad1.right_stick_y;
//                velocityLeft = gamepad1.left_stick_y;
//            }


            // Drive controls
            if(halfSpeed){
                setDriveSpeed(scaleGamepadInput(gamepad1.right_stick_y, SLOW_MODE_SCALE),
                        scaleGamepadInput(-gamepad1.left_stick_y, SLOW_MODE_SCALE));
                telemetry.addData("Reversed", "Yes!");
                telemetry.addData("Speed" , SLOW_MODE_SCALE);
                telemetry.addData("Voltage", driveVoltage);
            }
            else{
                setDriveSpeed(scaleGamepadInput(-gamepad1.left_stick_y, 1),
                        scaleGamepadInput(-gamepad1.right_stick_y, -1));
                telemetry.addData("Voltage", driveVoltage);
            }


            // Manipulator Controls
            if(gamepad1.left_trigger > 0.1) {
                manipulator.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                manipulator.setPower(gamepad1.right_trigger);
            } else manipulator.setPower(0);

            if(gamepad1.b) SLOW_MODE_SCALE = 0.4;
            if(gamepad1.x) SLOW_MODE_SCALE = 0.6;


            // GAMEPAD 2
            // Lift controls
            if (gamepad2.dpad_up){
                if (runtime.seconds() > 60)
                    holdBallHugger();
                lift1.setPower(-LIFT_SPEED);
                lift2.setPower(-LIFT_SPEED);
            } else if (gamepad2.dpad_down) {
                lift1.setPower(LIFT_SPEED);
                lift2.setPower(LIFT_SPEED);
            } else {
                lift1.setPower(0);
                lift2.setPower(0);
            }

            if (gamepad2.a)
                releaseBallHugger();

            if (gamepad2.x)
                holdBallHugger();


            if (gamepad2.dpad_left || gamepad2.b)
                gate.setPosition(GATE_DOWN);
            else gate.setPosition(GATE_UP);

            // rapid fire
            if (gamepad2.right_trigger > 0.3) {
                if(rapidReady) {
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            rapidReady = false;
                            while (gamepad2.right_trigger > 0.3) {
                                try {
                                    shootBall();
                                    simpleWait(500);
                                    gate.setPosition(GATE_DOWN);
                                    simpleWait(500);
                                    gate.setPosition(GATE_UP);
                                    simpleWait(700);
                                    idle();
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }

                            }
                            rapidReady = true;
                        }
                    }).start();
                }
            }


            telemetry.update();
            idle();
        }
        stopDriveMotors();
    }


    public void initHardware() {
        leftDrive1 = hardwareMap.dcMotor.get("l");
        leftDrive2 = hardwareMap.dcMotor.get("l2");
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive1 = hardwareMap.dcMotor.get("r");
        rightDrive2 = hardwareMap.dcMotor.get("r2");
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");

        shooter = hardwareMap.dcMotor.get("shoot");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manipulator = hardwareMap.dcMotor.get("manip");
        manipulator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        leftColorSensor = hardwareMap.colorSensor.get("left");
        leftColorSensor.setI2cAddress(I2cAddr.create8bit(0x2a));
        rightColorSensor = hardwareMap.colorSensor.get("right");
        rightColorSensor.setI2cAddress(I2cAddr.create8bit(0x2c));
        leftColorSensor.enableLed(false);
        rightColorSensor.enableLed(false);

        huggerHolderLeft = hardwareMap.servo.get("hugL");
        huggerHolderRight = hardwareMap.servo.get("hugR");
        gate = hardwareMap.servo.get("gate");



        //beam = new BreakBeamSensor(hardwareMap.digitalChannel.get("rec"), hardwareMap.digitalChannel.get("trans"));
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
        setMotorSpeeds(speed, rightDrive1, rightDrive2);
    }

    public void setLeftDriveSpeed(double speed) {
        setMotorSpeeds(speed, leftDrive1, leftDrive2);
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
        return new DcMotor[]{leftDrive1, leftDrive2, rightDrive1, rightDrive2};
    }

    // Servo Helper Methods
    public void setServoPositionWithTime(final Servo servo, final double targetPosition, final int timeMS) {
        setServoPositionWithTime(servo, targetPosition, timeMS, 50);
    }

    public void holdBallHugger() {
        huggerHolderLeft.setPosition(HUGGER_LEFT_DOWN);
        huggerHolderRight.setPosition(HUGGER_RIGHT_DOWN);
    }

    public void releaseBallHugger() {
        huggerHolderLeft.setPosition(HUGGER_LEFT_UP);
        huggerHolderRight.setPosition(HUGGER_RIGHT_UP);
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

    private static long nextPoll = System.currentTimeMillis() + 1000;
    private static double driveVoltage;
    public double scaleGamepadInput(double power, double scale){
        // Scale gamepad joystick movement in a nonlinear fashion
        if(Math.abs(power) <= 0.05)// Make sure joystick is actually being moved
            return 0;
        // clamp value between -1 and 1, the min and max values for joystick movement
        if(System.currentTimeMillis() > nextPoll) {
            driveVoltage = (hardwareMap.voltageSensor.get("l2 l").getVoltage() + hardwareMap.voltageSensor.get("r r1").getVoltage()) / 2;
            nextPoll = System.currentTimeMillis() + 1000;
        }

        if(driveVoltage < 12 && !halfSpeed)
            power = power * 1.15;
        else if (driveVoltage < 13 && !halfSpeed)
            power = power * 1.05;

        power = Range.clip(power, -1, 1);
        int index = (int) Range.clip((int) (Math.abs(power) * values.length-1), 0, values.length-1); // Clamp index between 0 and 16
        return power < 0 ? -values[index] * scale : values[index] * scale; // Return negative value if power is < 0
    }


    // AUTONOMOUS METHODS
    public void shootBall() throws InterruptedException {
        int startPosition = shooter.getCurrentPosition();
        shooter.setPower(-SHOOTER_SPEED);
        long endTime = System.currentTimeMillis() + 3000;
        int pos = shooter.getCurrentPosition();
        while (Math.abs(pos - startPosition) < 1440 && endTime > System.currentTimeMillis() && opModeIsActive()) {
            pos = shooter.getCurrentPosition();
            telemetry.addData("Shooter", Math.abs(pos-startPosition)/1440.0 +"%");
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