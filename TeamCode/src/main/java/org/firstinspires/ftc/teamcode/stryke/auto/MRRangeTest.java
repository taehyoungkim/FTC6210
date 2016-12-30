package org.firstinspires.ftc.teamcode.stryke.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.stryke.MRRangeSensor;

@Autonomous(name = "Range Test")
public class MRRangeTest extends LinearOpMode {

    public MRRangeSensor leftRangeSensor, rightRangeSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftRangeSensor = new MRRangeSensor(hardwareMap.i2cDevice.get("left"), I2cAddr.create8bit(0x34));
        rightRangeSensor = new MRRangeSensor(hardwareMap.i2cDevice.get("right"), I2cAddr.create8bit(0x36));


        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("left", leftRangeSensor.getUltraSonicDistance());
            telemetry.addData("right", rightRangeSensor.getUltraSonicDistance());
            telemetry.update();
            idle();
        }
    }
}
