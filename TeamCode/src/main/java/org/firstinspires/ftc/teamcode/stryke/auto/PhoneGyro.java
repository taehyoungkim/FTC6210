package org.firstinspires.ftc.teamcode.stryke.auto;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.teamcode.stryke.teleop.StrykeOpMode;

public class PhoneGyro implements SensorEventListener {
    private SensorManager sensorManager;
    private Sensor sensor;
    private float heading;

    public PhoneGyro(Context context) {
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    }


    @Override
    public final void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public final void onSensorChanged(SensorEvent event) {
        this.heading = event.values[0];
    }

    public double getHeading() {
        return heading * (180 / Math.PI);

    }


}
