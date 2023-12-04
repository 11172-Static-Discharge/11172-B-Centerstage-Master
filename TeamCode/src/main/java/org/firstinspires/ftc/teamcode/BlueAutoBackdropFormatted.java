/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.VisionCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Blue_Auto_Backdrop", group = "autos")
public class BlueAutoBackdropFormatted extends LinearOpMode {
    public VisionCamera camera;
    public double middlePos = 0.75;
    public double boardPos = 0.675;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        String path = "middle";
        camera = new VisionCamera(hardwareMap, telemetry);

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-37.97, -61.48, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-35.87, -29.75, Math.toRadians(89.17)))
                .lineTo(new Vector2d(-36.39, -40.53))
                .lineToLinearHeading(new Pose2d(-81.5, -41.32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-37.97, -61.48, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-37.97, -48.41, Math.toRadians(78.019108272)))
                .lineToLinearHeading(new Pose2d(-32.3204, -37.6256, Math.toRadians(52.790445864)))
                .lineToLinearHeading(new Pose2d(-37.97, -48.41, Math.toRadians(78.019108272)))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right.end())
                .lineToLinearHeading(new Pose2d(-81.5, -33, Math.toRadians(180.00)))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-37.97, -61.48, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-59.3, -36.78, Math.toRadians(89.17)))
                .lineTo(new Vector2d(-59.3, -51.53))
                .lineToLinearHeading(new Pose2d(-88, -39.32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-60.77, -36.39, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-47.28, -36.05))
                .lineTo(new Vector2d(-58.99, -60.62))
                .build();

        while (!isStarted()) {
            camera.telemetryTfod();
            slide.launch(false);
            slide.middleClaw();
            path = camera.getSide();
        }

        waitForStart();
        sleep(500);
        slide.bottomAuto();
        drive.setPoseEstimate(middle.start());

        switch (path) {
            case "left":
                drive.followTrajectorySequence(left);
                sleep(1000);
                slide.setArmPos(middlePos);
                sleep(2000);
                slide.openClaw();
                sleep(2000);
                drive.setPoseEstimate(park.start());
                slide.setArmPos(boardPos);
                drive.followTrajectorySequence(park);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(right2);
                sleep(1000);
                slide.setArmPos(middlePos);
                sleep(2000);
                slide.openClaw();
                sleep(2000);
                drive.setPoseEstimate(park.start());
                slide.setArmPos(boardPos);
                drive.followTrajectorySequence(park);
                break;
            case "middle":
                drive.followTrajectorySequence(middle);
                sleep(1000);
                slide.setArmPos(middlePos);
                sleep(2000);
                slide.openClaw();
                sleep(2000);
                drive.setPoseEstimate(park.start());
                slide.setArmPos(boardPos);
                drive.followTrajectorySequence(park);
                break;
        }
    }
}*/
