package org.firstinspires.ftc.teamcode.stryke.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.stryke.AnalogDistanceFinder;

@Autonomous(name = "Analog Tester")
public class AnalogDistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogDistanceFinder dist = (AnalogDistanceFinder) hardwareMap.analogInput.get("dist");
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("AnalogRead",dist.getVoltage());
            telemetry.addData("Distance", dist.distanceCM());
            telemetry.update();
        }
    }
}
