package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.stryke.GamepadListener;

@Autonomous(name = "Ball Shooter")
public class ShootFar extends StrykeAutonomous {

    static int balls = 2;
    static double speed = 0.5;
    static boolean shouldGoBack = false;
    static int selected = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Ready!");
        telemetry.update();



        GamepadListener gp1 = new GamepadListener(gamepad1);
        gp1.setOnPressed(GamepadListener.Button.DPAD_UP, new Runnable() {
            @Override
            public void run() {
                selected = (selected + 1) % 3;
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_DOWN, new Runnable() {
            @Override
            public void run() {
                selected = Math.abs(2 - selected) % 3;
            }
        });


        gp1.setOnPressed(GamepadListener.Button.DPAD_LEFT, new Runnable() {
            @Override
            public void run() {
                switch (selected) {
                    case(0) : balls = 1; break;
                    case(1) : speed = Range.clip(speed - 0.05, 0.1, 0.7); break;
                    case(2) : shouldGoBack = !shouldGoBack;
                }
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_RIGHT, new Runnable() {
            @Override
            public void run() {
                switch (selected) {
                    case(0) : balls = 2; break;
                    case(1) : speed = Range.clip(speed + 0.05, 0.1, 0.7); break;
                    case(2) : shouldGoBack = !shouldGoBack;
                }
            }
        });

        while(!isStarted() && !isStopRequested()) {
            gp1.update(gamepad1);
            telemetry.addData("Status", "Configure");
            telemetry.addData((selected == 0?" *":"") +"Num balls", balls + "");
            telemetry.addData((selected == 1?" *":"") +"Speed", speed + "");
            telemetry.addData((selected == 2?" *":"") + (shouldGoBack ? "Returning after shoot" : "Staying after shoot"), "");
            telemetry.update();
            idle();
        }

        waitForStart();
        runtime.reset();
        double dist = 24 * 3 * Math.sqrt(2);
        encoderDrive(dist, speed);
        simpleWaitS(2);
        if(balls == 2)
            shootTwoBalls();
        else shootBall();

        if(shouldGoBack)
            encoderDrive(-dist, speed);

    }

}
