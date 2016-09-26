package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Turn Test!")
public class AutonomousMotorTest extends StrykeOpMode {


    int wheelDiam = 6;
    private int encoderPPR = 7 * 40;

    DcMotor m;

    @Override
    public void runOpMode() throws InterruptedException {

//        leftDriveFront = hardwareMap.dcMotor.get("fl");
//        rightDriveFront = hardwareMap.dcMotor.get("fr");
//        leftDriveBack = hardwareMap.dcMotor.get("bl");
//        rightDriveBack = hardwareMap.dcMotor.get("br");

        leftDriveFront = hardwareMap.dcMotor.get("fl");
        rightDriveFront = hardwareMap.dcMotor.get("fr");
        leftDriveBack = hardwareMap.dcMotor.get("bl");
        rightDriveBack = hardwareMap.dcMotor.get("br");



        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER,getDriveMotors());
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, getDriveMotors());

        waitForStart();

        encoderDrive(4 * (24 * Math.sqrt(2)) - 2, 0.2, getDriveMotors());
        Thread.sleep(500);
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
        encoderTurn(45, -0.2, getDriveMotors());
        Thread.sleep(500);
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
        encoderDrive(12,-0.2, getDriveMotors());

    }

    public void encoderDrive(double inches, double speed, DcMotor... motors) throws InterruptedException {
        int pulses = (int) ((inches / (6 * Math.PI) * 280) * 1.6);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        while(getAverageEncoderPosition(motors) <= pulses) {
            setDriveSpeed(speed, -speed);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", getAverageEncoderPosition(motors));
            telemetry.update();
            idle();
        }

        setDriveSpeed(0, 0);


    }

    public void encoderTurn(double deg, double speed, DcMotor... motors) throws InterruptedException {
        //                    Arc%     * Circumference  / Dist. per rotation * ppr * percent error
        int pulses = (int) ((((deg/360) * (18 * Math.PI) / (6 * Math.PI) * 280) * 1.6) * 1.55);
        resetMotorEncoders();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        while(getAverageEncoderPosition(motors) <= pulses) {
            setDriveSpeed(speed, speed);
            telemetry.addData("Target", pulses);
            telemetry.addData("Current", getAverageEncoderPosition(motors));
            telemetry.update();
            idle();
        }

        setDriveSpeed(0, 0);

    }

    public void pidEncoderTurn(int deltaP, double speed) throws InterruptedException {

         while(m.isBusy() || Math.abs(m.getCurrentPosition()) < deltaP) {
             telemetry.addData("Encoder Value", m.getCurrentPosition());
             m.setPower(speed);
             idle();
         }
        m.setPower(0);




    }

}

