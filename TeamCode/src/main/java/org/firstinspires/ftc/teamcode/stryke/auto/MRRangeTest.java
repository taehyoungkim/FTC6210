package org.firstinspires.ftc.teamcode.stryke.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.stryke.MRRangeSensor;

@Autonomous(name = "Range Test")
public class MRRangeTest extends LinearOpMode {

    public MRRangeSensor hex34, hex36;

    @Override
    public void runOpMode() throws InterruptedException {
        hex34 = new MRRangeSensor(hardwareMap.i2cDevice.get("34"), I2cAddr.create8bit(0x34));
        hex36 = new MRRangeSensor(hardwareMap.i2cDevice.get("36"), I2cAddr.create8bit(0x36));

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("0x34", hex34.distanceCm());
            telemetry.addData("0x36", hex36.distanceCm());
            telemetry.update();
            idle();
        }
    }
}
