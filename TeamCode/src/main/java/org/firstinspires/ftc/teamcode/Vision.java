package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Vision")
public class Vision extends LinearOpMode {
    @Override
        public void runOpMode() throws InterruptedException {

            int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

            AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                    .build();

            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor1)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
//                    .setCameraMonitorViewId(portalsList[0])
                    .build();

            VisionPortal visionPortal2 = new VisionPortal.Builder()
                    .addProcessor(tagProcessor1)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
//                    .setCameraMonitorViewId(portalsList[1])
                    .build();

            waitForStart();

            while (!isStopRequested() && opModeIsActive()) {
                visionPortal.getProcessorEnabled(tagProcessor1);
                visionPortal.close();
                visionPortal.stopStreaming();
                visionPortal.resumeStreaming();
                visionPortal.stopLiveView();
                visionPortal.resumeLiveView();
                visionPortal.saveNextFrameRaw("Test");
                visionPortal.getActiveCamera();
                tagProcessor1.getPerTagAvgPoseSolveTime();
                if (tagProcessor1.getDetections().size() > 0) {
                    AprilTagDetection tag = tagProcessor1.getDetections().get(0);

                    

                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);
                }
                telemetry.update();

        }
    }
}
