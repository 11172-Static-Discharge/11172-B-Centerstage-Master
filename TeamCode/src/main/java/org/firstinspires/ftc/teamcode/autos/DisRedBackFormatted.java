package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.VisionCamera;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "DisRedBackFormatted", group = "autos")
public class DisRedBackFormatted extends LinearOpMode {



    private VisionCamera camera;



    public double middlePos1 = 0.75;
    public int offset = 0;
    public double xOffset = -2;
    public double yOffset = 0;

    public double rightOffset = 0;
    public int sign = 1;

    public double dropXPos = -20  + xOffset;
    public int dropLiftPos = -1900;

    public boolean leftPark = true;
    public double tapeOffset = 0;

    public double pickup = Lift.wristPickup;
    public double hover = Lift.wristHover;

    public double dispense = Lift.dispenseDrop;
    public double launch = Lift.dispenseLaunch;



    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(telemetry, hardwareMap);
        String path = "middle";
        camera = new VisionCamera(hardwareMap, telemetry, "RED");



        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(-13.4, -43.3))
                .build();//TO TAPE

        TrajectorySequence middle2 = drive.trajectorySequenceBuilder(middle.end())
                .lineTo(new Vector2d(-52,  -19.4),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//GO TO BACKBOARD

        //har har har ah r arh a

        TrajectorySequence middle3 = drive.trajectorySequenceBuilder(middle2.end())
                .lineTo(new Vector2d(-35,  -18),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//REVERSE FROM BOARD

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(0, -29.6),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(12, -29.6),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(2,  -29.6),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//TAPE


        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left.end())
                .lineTo(new Vector2d(-25,  -28),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-40,  -26),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//GO TO BOARD

        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineTo(new Vector2d(-32,  -28))
                .build();//REVERSE FROM BOARD



        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                //.lineTo(new Vector2d(-17, -35),
                  //      SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                    //    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-21, -35),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-7, -35),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-21.34, -35),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//TAPE

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right.end())
                .lineTo(new Vector2d(-30,  -13),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-42, -13),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();//GO TO BOARD

        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .lineTo(new Vector2d(-32,  -15))
                .build();//REVERSE FROM BOARD


        while (!isStarted()) {
            camera.telemetryTfod();
            telemetry.addData("path: ", path);
            telemetry.addData("getSide: ", camera.getSide());
            telemetry.update();
            lift.setRightClaw(true);
            lift.setLeftClaw(true);

            path = camera.getSide();
            if(gamepad1.dpad_right) path = "right";
            if(gamepad1.dpad_left)  path = "left";

            if(gamepad1.dpad_left) leftPark = true;
            else if(gamepad1.dpad_right) leftPark = false;
        }

        waitForStart();

        sleep(500);

        drive.setPoseEstimate(middle.start());

        switch(path)
        {
            case "left":
                drive.followTrajectorySequence(left);
                lift.setWristPosFixed(pickup);
                sleep(700);
                lift.setRightClaw(false);
                sleep(500);
                lift.setWristPosFixed(hover);
                drive.followTrajectorySequence(left2);
                lift.setDispenser(dispense);
                sleep(2000);
                lift.setDispenser(launch);
                drive.followTrajectorySequence(left3);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                lift.setWristPosFixed(pickup);
                sleep(700);
                lift.setRightClaw(false);
                sleep(500);
                lift.setWristPosFixed(hover);
                drive.followTrajectorySequence(right2);
                lift.setDispenser(dispense);
                sleep(2000);
                lift.setDispenser(launch);
                drive.followTrajectorySequence(right3);
                break;
            case "middle":
                drive.followTrajectorySequence(middle);
                sleep(500);
                lift.setWristPosFixed(pickup);
                sleep(1300);
                lift.setRightClaw(false);
                sleep(500);
                lift.setWristPosFixed(hover);
                sleep(500);
                drive.followTrajectorySequence(middle2);
                sleep(1000);
                lift.setDispenser(dispense);
                sleep(2000);
                lift.setDispenser(launch);
                drive.followTrajectorySequence(middle3);
                break;
        }

        Pose2d myPose = drive.getPoseEstimate();

       /* TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(myPose)
                .lineTo(new Vector2d(-24, 36))
                .lineTo(new Vector2d(-24, 9))
                .lineTo(new Vector2d(-36, 9))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(myPose)
                .lineTo(new Vector2d(-24, 36))
                .lineTo(new Vector2d(-24, 69))
                .lineTo(new Vector2d(-36, 69))
                .build();


        drive.followTrajectorySequence(leftPark ? parkLeft : parkRight);*/

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
            lift.moveToPower(targetPos, power, false);
            lift.setLeftClaw(clawL);
            lift.setRightClaw(clawR);
            lift.setWristPosFixed(wristPos);
        }
    }
}
