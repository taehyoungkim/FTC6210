package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.stryke.GamepadListener;

@Autonomous(name = "Ball Shooter")
public class ShootFar extends StrykeAutonomous {

    static int balls = 2;
    static double speed = 0.5;
    static boolean shouldGoBack = false, hitCap = false, park = false, enabled = true, far = true;
    static int selected = 0;
    static int waitTime = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Ready!");
        telemetry.update();



        GamepadListener gp1 = new GamepadListener(gamepad1);
        gp1.setOnPressed(GamepadListener.Button.A, new Runnable() {
            @Override
            public void run() {
                enabled = !enabled;
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_UP, new Runnable() {
            @Override
            public void run() {
                if(!enabled) return;
                selected = selected - 1;
                if(selected < 0) selected = 6;
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_DOWN, new Runnable() {
            @Override
            public void run() {
                if(!enabled) return;
                selected = (selected + 1) % 6;
            }
        });


        gp1.setOnPressed(GamepadListener.Button.DPAD_LEFT, new Runnable() {
            @Override
            public void run() {
                if(!enabled) return;
                switch (selected) {
                    case(0) : balls = balls == 1 ? 2: 1; break;
                    case(1) : speed = Range.clip(speed - 0.05, 0.1, 0.7); break;
                    case(2) : shouldGoBack = !shouldGoBack;if(shouldGoBack) park = false; break;
                    case(3) : waitTime -= 1;break;
                    case(4) : hitCap = !hitCap;break;
                    case(5) : park = !park; if(park) shouldGoBack = false;break;
                    case(6) : far = !far;break;
                }
            }
        });

        gp1.setOnPressed(GamepadListener.Button.DPAD_RIGHT, new Runnable() {
            @Override
            public void run() {
                if(!enabled) return;
                switch (selected) {
                    case(0) : balls = balls == 1 ? 2: 1; break;
                    case(1) : speed = Range.clip(speed + 0.05, 0.1, 0.7); break;
                    case(2) : shouldGoBack = !shouldGoBack;if(shouldGoBack) park = false; break;
                    case(3) : waitTime += 1;break;
                    case(4) : hitCap = !hitCap;break;
                    case(5) : park = !park; if(park) shouldGoBack = false;break;
                    case(6) : far = !far;break;
                }
            }
        });

        while(!isStarted() && !isStopRequested()) {
            gp1.update(gamepad1);
            telemetry.addData("Status", enabled ? "Configure." : "Configured.");
            telemetry.addData(getPrefix(0) +"Balls", balls + "");
            telemetry.addData(getPrefix(1) +"Speed", speed + "");
            telemetry.addData(getPrefix(2) + "Post-Shoot", (shouldGoBack ? "Return" : "Stay"));
            telemetry.addData(getPrefix(3) + "Wait time", waitTime+"");
            telemetry.addData(getPrefix(4) + "Snag cap", hitCap ? "Yes": "No");
            telemetry.addData(getPrefix(5) + "Park", park ? "Yes": "No");
            telemetry.addData(getPrefix(6) + "Starting Point", far ? "Far": "Close");
            telemetry.update();
            idle();
        }

        waitForStart();
        runtime.reset();

        if(waitTime < 0) waitTime = 0;
        simpleWaitS(waitTime);


        double dist;
        if(far)
            dist = 24 * 3 * Math.sqrt(2);
        else dist = 24 * 2.5;

        encoderDrive(dist, speed);
        simpleWaitS(0.2);
        if(balls == 2)
            shootTwoBalls();
        else shootBall();

        if(hitCap){
            if(park) {
                manip.setPower(-0.2);
                encoderDrive(5, 0.2);
                simpleWaitS(1);
                encoderDrive(5, 0.2);
            } else {
                manip.setPower(0.8);
                encoderDrive(5, 0.2);
                simpleWaitS(0.2);
                encoderDrive(10, -0.2);
                manip.setPower(0);
            }

        }


        if(!park && shouldGoBack)
            encoderDrive(dist, - speed);

    }

    public String getPrefix(int ind) {
        return (enabled ? (selected == ind?" *":"") : "");
    }

}
