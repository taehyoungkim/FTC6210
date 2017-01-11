package org.firstinspires.ftc.teamcode.stryke.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.stryke.AnalogDistanceFinder;

@Autonomous(name = "Beta Red")
public class BetaRedAuto extends MainRedAuto {

    public OpticalDistanceSensor wall;

    @Override
    public void initHardware() {
        super.initHardware();
        wall = hardwareMap.opticalDistanceSensor.get("wall");
    }

    @Override
    public void driveUntilStop(double speed) throws InterruptedException {
        setDriveSpeed(0.5, -0.5);
        while(wall.getLightDetected() < 10 && opModeIsActive()) {
            if(isStopRequested()) return;
        }
        stopDriveMotors();


    }


}
