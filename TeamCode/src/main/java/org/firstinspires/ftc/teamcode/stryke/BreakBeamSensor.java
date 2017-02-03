package org.firstinspires.ftc.teamcode.stryke;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class BreakBeamSensor {

    private DigitalChannel sensor, light;

    public BreakBeamSensor(DigitalChannel sensor, DigitalChannel light){
        this.sensor = sensor;
        this.light = light;
        turnOn();
    }

    public void turnOn() {
        light.setState(true);
    }

    public void turnOff() {
        light.setState(false);
    }

    public void toggle() {
        light.setState(!isOn());
    }

    public boolean isBroken(){
        return sensor.getState();
    }

    public boolean isOn() {
        return light.getState();
    }
}
