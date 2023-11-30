package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoRITD", group = "autos")
public class AutoRITD extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Lift lift = new Lift(telemetry, hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-12.00, -63.23, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-12.00, -39.75))
                .lineTo(new Vector2d(-12.37, -38.85))
                .lineTo(new Vector2d(-34.11, -56.21))
                .lineTo(new Vector2d(-60.42, -60.60))
                .build();
        drive.setPoseEstimate(middle.start());


        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-12.37, -63.23, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-0.50, -33.00))
                .lineTo(new Vector2d(-0.50, -40.00))
                .lineTo(new Vector2d(-35.17, -40.00))
                .lineTo(new Vector2d(-40.78, -61.30))
                .lineTo(new Vector2d(-60.95, -62.00))
                .build();
        drive.setPoseEstimate(right.start());

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-12.37, -63.23, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-23.50, -33.00))
                .lineTo(new Vector2d(-23.50, -40.00))
                .lineTo(new Vector2d(-35.17, -44.81))
                .lineTo(new Vector2d(-40.78, -61.30))
                .lineTo(new Vector2d(-60.95, -62.00))
                .build();
        drive.setPoseEstimate(left.start());

        lift.closeClaw();

        waitForStart();
        drive.followTrajectorySequence(middle);
        lift.setArmPower(0.25, -1);
        sleep(1000);
        lift.openClaw();
    }
}
