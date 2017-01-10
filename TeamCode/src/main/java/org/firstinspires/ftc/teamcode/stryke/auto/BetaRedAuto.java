package org.firstinspires.ftc.teamcode.stryke.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.stryke.AnalogDistanceFinder;

@Autonomous(name = "Beta Red")
public class BetaRedAuto extends MainRedAuto {

    public AnalogDistanceFinder leftFinder, rightFinder;

    @Override
    public void initHardware() {
        super.initHardware();
//        leftFinder = (AnalogDistanceFinder)hardwareMap.analogInput.get("leftStick");
//        rightFinder = (AnalogDistanceFinder)hardwareMap.analogInput.get("rightStick");
        rightFinder.reversed = true;
    }

    @Override
    public void driveUntilStop(double speed) throws InterruptedException {
        double left = leftFinder.distanceCM(), right = rightFinder.distanceCM();
        // Left side is further away
        setDriveSpeed(speed, speed);
        while(left > right && opModeIsActive()) {
            left = leftFinder.distanceCM();
            right = rightFinder.distanceCM();
            statusTelemetry("Align left. L:" + left + " R:"+ right);
        }
        stopDriveMotors();

        // right side is further away
        setDriveSpeed(-speed, -speed);
        while(right > left && opModeIsActive()) {
            left = leftFinder.distanceCM();
            right = rightFinder.distanceCM();
            statusTelemetry("Align right. L:" + left + " R:"+ right);
        }
        stopDriveMotors();

        int targetDistance = 5;
        double average = (left + right )/2;
        setDriveSpeed(speed, -speed);
        while(average > targetDistance && opModeIsActive()){
            left = leftFinder.distanceCM();
            right = rightFinder.distanceCM();
            average = (left + right) / 2;
            statusTelemetry("Forward till " + targetDistance + ". L:" + left + " R:"+ right + " A:"+average);
        }
        stopDriveMotors();

    }


}
