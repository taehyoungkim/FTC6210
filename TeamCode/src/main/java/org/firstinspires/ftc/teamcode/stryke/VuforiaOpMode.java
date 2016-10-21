package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Taehyoung Kim on 10/17/16.
 */
@TeleOp(name="Camera", group="Linear Opmode")
public class VuforiaOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AR7o5TD/////AAAAGcuKig4A8ETxp/qfcDKmtsKBRHbcbvceGAt17spZ0wgSCki1SZyj8hSpbmVIdXVOBDFGQjFKDnxHOV3OTaQVnI0nGknHpLf8OMYCBrPUqiZFVjNRZ3XBHVQ3lg7mCZnyPHVsdtu83bab0TLqbdLFwonDrCK3wWSgAfKWYbTcdZFdieAMnTiuG1+K3jaeBv6LT7P+kl1yPnGDZx9J3XF9PpmwEW1jXYref5InPsCfyMnGxajRHa+j26mkQcTHMy1L3EUrsmFmYjyBlru1VrsZS97CjlH9xhUAmZ0iDTr8tBssGVXg8TDxq5pDXAHo26/fLod5ysXdDKnfo9cEal25DwV8o/9jxAB+HfPR+WGeWZVe";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


        VuforiaLocalizer vuforia  = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Translation: ", translation);
                    //if phone is mounted vertically.
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    //Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    telemetry.addData(beacon.getName() + "-Degrees: ", degreesToTurn);
                }
            }
            telemetry.update();
        }
    }

}
