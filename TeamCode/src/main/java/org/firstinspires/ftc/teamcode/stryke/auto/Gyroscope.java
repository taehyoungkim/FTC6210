package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

public class Gyroscope {


    private boolean isIMU;

    //IMU
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //MODERN ROBOTICS
    GyroSensor gyroSensor;


    public Gyroscope(String type, HardwareMap map) {
        if (type.equalsIgnoreCase("imu")) {
            isIMU = true;

            parameters                     = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = map.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } else {
            isIMU = false;
            gyroSensor = map.gyroSensor.get("gyro");
            gyroSensor.calibrate();
        }
    }

    public double getHeading() {
        if (isIMU) {
            double value = imu.getAngularOrientation().firstAngle * -1;
            if(imu.getAngularOrientation().firstAngle < -180)
                value -= 360;
            if (value < 0) value = 360 - Math.abs(value);
            return value;
        }
        else
            return gyroSensor.getHeading();

    }

    public void calibrate() {
        if(isIMU)
            imu.initialize(parameters);
        else
            gyroSensor.calibrate();
    }
}
