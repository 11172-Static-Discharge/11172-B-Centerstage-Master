package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "poopAuto", group = "autos")
public class BlueAutoTest extends LinearOpMode
{    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phonhie camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    /*
    things to fix for auto
    lower angular velocity
    tighten servo screw from arm to servo
    tighten servo screw to mount plate
    raise threshold for accuracy
    */
    private static final String[] labels = {"BlueElementv2", "RedElementv2"};
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/ModelMoreTraining.tflite";



    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public double middlePos1 = 0.75;


    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(telemetry, hardwareMap);
        String path = "middle";

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-4, 20))
                .lineTo(new Vector2d(-5, 22.25))
                .build();

        TrajectorySequence middle2 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-8.75, -5))
                .build();

        TrajectorySequence middleStack1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(0, 15.5))
                .lineTo(new Vector2d(30, 15.5))
                .build();

        TrajectorySequence middleStack2 = drive.trajectorySequenceBuilder(middleStack1.end())
                .lineTo(new Vector2d(50.75, 14.6))
                .build();

        TrajectorySequence middleStackScore = drive.trajectorySequenceBuilder(new Pose2d(50.75, 14.6, Math.toRadians(0)))
                .lineTo(new Vector2d(30, 15.5))
                .lineTo(new Vector2d(0, 15.5))
                .lineTo(new Vector2d(0, 0))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-37.97, -61.48, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-37.97, -48.41, Math.toRadians(78.019108272)))
                .lineToLinearHeading(new Pose2d(-32.3204, -37.6256, Math.toRadians(52.790445864)))
                .lineToLinearHeading(new Pose2d(-37.97, -48.41, Math.toRadians(78.019108272)))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-37.97, -61.48, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-58.5, -36.78, Math.toRadians(89.17)))
                .lineTo(new Vector2d(-58.5, -51.53))
                .lineToLinearHeading(new Pose2d(-88, -39.32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-60.77, -36.39, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-47.28, -36.05))
                .lineTo(new Vector2d(-58.99, -60.62))
                .build();
        initTfod();

        while (!isStarted()) {
            telemetryTfod();
            telemetry.update();
            lift.setRightClaw(true);
            lift.setLeftClaw(true);
            path = "middle";
        }


        waitForStart();

        sleep(500);

        drive.setPoseEstimate(middle.start());

        switch(path)
        {
            case "left":
                drive.followTrajectorySequence(left);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                break;
            case "middle":
                drive.followTrajectorySequence(middle);
                lift.setWristPosFixed(0.900);
                sleep(1000);
                lift.setLeftClaw(false);
                sleep(1000);
                drive.setPoseEstimate(middle2.start());
                lift.setWristPosFixed(0.42);
                drive.followTrajectorySequence(middle2);
                //drive.followTrajectorySequence(middle3);
                sleepLift(1000, lift, -1600, true, false, 0.42);
                sleepLift(250, lift, -1600, true, false, 0.42);
                sleepLift(250, lift, -1600, false, false, 0.42);
                sleepLift(1000, lift, 0, false, false, 0.15);
                drive.setPoseEstimate(middleStack1.start());
                drive.followTrajectorySequence(middleStack1);
                sleepLift(1000, lift, -190, false, false, 0.9);
                drive.followTrajectorySequence(middleStack2);
                sleepLift(2000, lift, -190, true, false, 0.9);
                sleepLift(1000, lift, 0, true, false, 0.15);
                drive.followTrajectorySequence(middleStackScore);
                sleepLift(1000, lift, -1600, true, false, 0.42);
                sleepLift(250, lift, -1600, true, false, 0.42);
                sleepLift(250, lift, -1600, false, false, 0.42);
                sleepLift(1000, lift, -200, false, false, 0.9);
                //sleepLift(1000, lift, -1520); hi jayandesh
                break;
        }


    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_ASSET)
                .setModelLabels(labels)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                //.setModelAspectRatio(16.0/9.0)

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.8f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private String getSide() {
        List<Recognition> recognition = tfod.getRecognitions();
        if (recognition.isEmpty()) return "left";
        for (int i = 0; i<recognition.size(); i++) {
            if (recognition.get(i).getWidth()>250 || recognition.get(i).getHeight()>250) {}
            else if (recognition.get(i).getLeft() > 300) return "right";
            else if (recognition.get(i).getLeft() <= 300) return "middle";
        }
        return "left";

    }

    private void sleepLift(int milliseconds, Lift lift, int targetPos, boolean clawR, boolean clawL, double wristPos)
    {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() <= milliseconds)
        {
            lift.moveTo(targetPos);
            lift.setLeftClaw(clawL);
            lift.setRightClaw(clawR);
            lift.setWristPosFixed(wristPos);
        }
    }
}