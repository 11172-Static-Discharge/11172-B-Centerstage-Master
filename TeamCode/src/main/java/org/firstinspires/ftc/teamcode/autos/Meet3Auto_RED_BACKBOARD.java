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

@Autonomous (name = "Meet3Auto_RED_BACKBOARD", group = "autos")
public class Meet3Auto_RED_BACKBOARD extends LinearOpMode
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
    public int offset = 0;
    public double xOffset = 0;
    public double yOffset = 0;

    public double rightOffset = -4.75;
    public int sign = -1;

    public double dropXPos = -24  + xOffset;
    public int dropLiftPos = -1900;

    public boolean leftPark = true;


    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(telemetry, hardwareMap);
        String path = "middle";

        initTfod();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-6 + xOffset + rightOffset, sign * 44 + yOffset))
                .build();

        TrajectorySequence middle2 = drive.trajectorySequenceBuilder(middle.end())
                .lineTo(new Vector2d(dropXPos - 2 + rightOffset, sign *35.75 + yOffset))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(0 + xOffset + rightOffset, sign *35 + yOffset))
                .lineTo(new Vector2d(12 + xOffset + rightOffset, sign *35 + yOffset))
                .lineTo(new Vector2d(4.1 + xOffset + rightOffset, sign *35 + yOffset))
                .build();

        /*TrajectorySequence rightSwitch1 = drive.trajectorySequenceBuilder(right.end())
                .lineTo(new Vector2d(-4 + xOffset + rightOffset, sign *35))
                .build();


        TrajectorySequence rightSwitch2 = drive.trajectorySequenceBuilder(rightSwitch1.end())
                .lineTo(new Vector2d(-4 + xOffset - 2 + rightOffset, sign *35))
                .lineTo(new Vector2d(-4 + xOffset - 2 + rightOffset, sign *(35 - 7)))
                .lineTo(new Vector2d(-4 + xOffset + rightOffset, sign *(35 - 7)))
                .build();*/

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right.end())
                .lineTo(new Vector2d(0 + xOffset + rightOffset, sign *38 + yOffset))
                .lineTo(new Vector2d(dropXPos - 1.75 + rightOffset, sign *(43.25 + 2 + 1.5)+ yOffset) )
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-19.5 - 2 + xOffset + rightOffset, sign *(31)+ yOffset) )
                .build();

        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left.end())
                .lineTo(new Vector2d(dropXPos - 5.5 + rightOffset, sign *(29)+ yOffset))
                .build();

        //hey jayan

        while (!isStarted()) {
            telemetryTfod();
            telemetry.update();
            lift.setRightClaw(true);
            lift.setLeftClaw(true);
            path = getSide();

            if(gamepad1.dpad_left) leftPark = true;
            else if(gamepad1.dpad_right) leftPark = false;

            lift.arm.reset();
        }

        waitForStart();

        sleep(500);

        drive.setPoseEstimate(middle.start());

        switch(path)
        {
            case "left":
                drive.followTrajectorySequence(left);
                lift.setWristPosFixed(0.900);
                sleep(1000);
                lift.setLeftClaw(false);
                sleep(1000);
                lift.setWristPosFixed(0.42);
                drive.followTrajectorySequence(left2);
                //drive.followTrajectorySequence(middle3);
                sleepLiftPower(1500, lift, dropLiftPos, true, true, 0.42, offset, 0.5);
                sleep(1000);
                sleepLiftPower(1000, lift, dropLiftPos, false, true, 0.42, offset, 0);
                //sleepLift(750, lift, dropLiftPos, false, false, 0.42, offset);
                sleepLift(1000, lift, -900, false, true, 0.42, offset);
                sleepLift(2000, lift, 0, true, true, 0.15, offset);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                lift.setWristPosFixed(0.900);
                sleep(1000);
                lift.setLeftClaw(false);
                sleep(1000);

               /* drive.followTrajectorySequence(rightSwitch1);
                lift.setRightClaw(false);
                sleep(1000);
                lift.setWristPosFixed(0.15);
                drive.followTrajectorySequence(rightSwitch2);
                sleep(100);
                lift.setWristPosFixed(0.900);
                sleep(1000);
                lift.setLeftClaw(true);
                sleep(500);*/

                lift.setWristPosFixed(0.42);

                drive.followTrajectorySequence(right2);
                //drive.followTrajectorySequence(middle3);
                sleepLiftPower(1500, lift, dropLiftPos, true, true, 0.42, offset, 0.5);
                sleep(1000);
                sleepLiftPower(1000, lift, dropLiftPos, false, true, 0.42, offset, 0);
                //sleepLift(750, lift, dropLiftPos, false, false, 0.42, offset);
                sleepLift(1000, lift, -900, false, true, 0.42, offset);
                sleepLift(2000, lift, 0, true, true, 0.15, offset);
                break;
            case "middle":
                drive.followTrajectorySequence(middle);
                lift.setWristPosFixed(0.900);
                sleep(1000);
                lift.setLeftClaw(false);
                sleep(1000);
                lift.setWristPosFixed(0.42);
                drive.followTrajectorySequence(middle2);
                //drive.followTrajectorySequence(middle3);
                sleepLiftPower(1500, lift, dropLiftPos, true, true, 0.42, offset, 0.5);
                sleep(1000);
                sleepLiftPower(1000, lift, dropLiftPos, false, true, 0.42, offset, 0);
                //sleepLift(750, lift, dropLiftPos, false, false, 0.42, offset);
                sleepLift(1000, lift, -900, false, true, 0.42, offset);
                sleepLift(2000, lift, 0, true, true, 0.15, offset);
                break;
        }

        Pose2d myPose = drive.getPoseEstimate();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(myPose)
                .lineTo(new Vector2d(-24, sign*36))
                .lineTo(new Vector2d(-24, sign*9))
                .lineTo(new Vector2d(-36, sign*9))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(myPose)
                .lineTo(new Vector2d(-24, sign*36))
                .lineTo(new Vector2d(-24, sign*69))
                .lineTo(new Vector2d(-36, sign*69))
                .build();


        drive.followTrajectorySequence(leftPark ? parkLeft : parkRight);

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
            else if (recognition.get(i).getLeft() > 275) return "middle";
            else if (recognition.get(i).getLeft() <= 275) return "right";
        }
        return "left";

    }

    private void sleepLift(int milliseconds, Lift lift, int targetPos, boolean clawR, boolean clawL, double wristPos, int offset)
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

    private void sleepLiftPower(int milliseconds, Lift lift, int targetPos, boolean clawR, boolean clawL, double wristPos, int offset, double power)
    {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() <= milliseconds)
        {
            lift.moveToPower(targetPos, power);
            lift.setLeftClaw(clawL);
            lift.setRightClaw(clawR);
            lift.setWristPosFixed(wristPos);
        }
    }
}