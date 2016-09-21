package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class RobotHardware {

    private HardwareMap map;
    private boolean hardwareInitialized, gyroInitialized;

    // Drive Motors
    private DcMotor leftDriveFront, rightDriveFront, leftDriveBack, rightDriveBack;
    private GyroSensor gyro;

    public RobotHardware(HardwareMap map) {
        this.map = map;
        hardwareInitialized = false;
    }

    public void initHardware() {
        if(hardwareInitialized) {
            DbgLog.msg("RobotHardware", "Attempted to initialize hardware more than once!");
            return;
        }

        leftDriveFront = map.dcMotor.get("lf");
        rightDriveFront = map.dcMotor.get("rf");
        leftDriveBack = map.dcMotor.get("lb");
        rightDriveBack = map.dcMotor.get("rb");

//        gyro = findHardwareDevice(map.gyroSensor, "gyro");
//        calibrateGyro();

        hardwareInitialized = true;
    }

    public void calibrateGyro() {
        if(gyroInitialized || gyro == null) return;
        new Thread(new Runnable() {
            @Override
            public void run() {
                gyro.calibrate();
                try {
                    while(gyro.isCalibrating())
                        Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                gyroInitialized = true;
            }
        }).start();
    }

    public boolean isGyroInitialized() {
        return gyroInitialized;
    }

    public boolean isHardwareInitialized() {
        return hardwareInitialized;
    }

    public <T> HardwareDevice findHardwareDevice(HardwareMap.DeviceMapping<HardwareDevice> devices, String name) {
        try {
            return devices.get(name);
        } catch (IllegalArgumentException e){
            return null;
        }
    }

    // **** HELPER METHODS ****
    // DcMotor Helper Methods
    public void stopDriveMotors() {
        setDriveSpeed(0,0);
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
        if(!isHardwareInitialized()) initHardware();
        setMotorSpeeds(speed, rightDriveBack, rightDriveFront);
    }

    public void setLeftDriveSpeed(double speed) {
        if(!isHardwareInitialized()) initHardware();
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
        for(DcMotor motor : motors){
            if(ignoreSign)
                total += Math.abs(motor.getCurrentPosition());
            else
                total += motor.getCurrentPosition();
        }
        return total/motors.length;
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

    // **** GETTERS ****
    public DcMotor getLeftDriveFront() {
        return leftDriveFront;
    }

    public DcMotor getRightDriveFront() {
        return rightDriveFront;
    }

    public DcMotor getLeftDriveBack() {
        return leftDriveBack;
    }

    public DcMotor getRightDriveBack() {
        return rightDriveBack;
    }

    public GyroSensor getGyro() {
        if(isGyroInitialized())
            return gyro;
        return null;
    }
}