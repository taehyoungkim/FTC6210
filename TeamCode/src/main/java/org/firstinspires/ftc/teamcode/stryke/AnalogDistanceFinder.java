package org.firstinspires.ftc.teamcode.stryke;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

public class AnalogDistanceFinder extends AnalogInput {

    public double rodLength = 30;
    public boolean reversed = false;

    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public AnalogDistanceFinder(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    private static int VOLTAGE_MAX = 1023, VOLTAGE_MIN = 0;
    public double distanceCM() {
        double angle;
        if(reversed) {
            angle = map(VOLTAGE_MAX - getVoltage(), VOLTAGE_MIN, VOLTAGE_MAX, 0, 90);
        } else angle = map(getVoltage(), VOLTAGE_MIN, VOLTAGE_MAX, 0, 90);

        return rodLength * Math.cos(angle);
    }

    private double map(double value, double l1, double h1, double l2, double h2) {
        double r1 = h1 - l1;
        double r2 = h2 - l2;
        double p = (value - l1)/r1;
        return (p * r2) + l2;
    }

}
