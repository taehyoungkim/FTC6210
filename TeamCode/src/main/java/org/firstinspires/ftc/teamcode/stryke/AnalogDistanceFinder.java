package org.firstinspires.ftc.teamcode.stryke;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

public class AnalogDistanceFinder {

    public double rodLength = 10;
    public boolean reversed = false;
    private AnalogInput device;

    public AnalogDistanceFinder(AnalogInput device) {
        this.device = device;
    }

    int VOLTAGE_MAX = 5, VOLTAGE_MIN = 0, ANGLE_MAX = 300 - 5, ANGLE_MIN = 0;

    public double distanceCM() {
        double angle;
        if(reversed) {
            angle = map(VOLTAGE_MAX - device.getVoltage(), VOLTAGE_MIN, VOLTAGE_MAX, ANGLE_MIN, ANGLE_MAX);
        } else angle = map(device.getVoltage(), VOLTAGE_MIN, VOLTAGE_MAX, ANGLE_MIN, ANGLE_MAX);

        return rodLength * Math.cos(angle * Math.PI / 180);
    }

    public double getAngle(){
        return  map(device.getVoltage(), VOLTAGE_MIN, VOLTAGE_MAX, ANGLE_MIN, ANGLE_MAX);
    }

    public double getVoltage() {
        return device.getVoltage();
    }

    public AnalogInput getDevice() {
        return device;
    }

    public double map(double value, double l1, double h1, double l2, double h2) {
        double r1 = h1 - l1;
        double r2 = h2 - l2;
        double p = (value - l1)/r1;
        return (p * r2) + l2;
    }

}
