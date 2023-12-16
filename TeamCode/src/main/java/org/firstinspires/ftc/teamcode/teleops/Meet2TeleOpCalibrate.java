package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "Meet2TeleOpCali", group = "drive")
public class Meet2TeleOpCalibrate extends LinearOpMode {

    public double[] hover = new double[]{0.1, 0.9, 0.9};
    public double[] pickup = new double[]{0.12, 0.88, 0.93};
    public double[] drop1 = new double[]{0.58, 0.42, 0.34};
    public double[] drop2 = new double[]{0.58, 0.42, 0.34};
    public double[] drop3 = new double[]{0.6, 0.4, 0.38};
    public double speed = 0.5;



    public boolean rClosed, lClosed;
    public boolean positionSet, interpolate;

    public int liftPos;
    public double wristPos;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Slide slide = new Slide(telemetry, hardwareMap);

        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide");
        DcMotorEx winch = hardwareMap.get(DcMotorEx.class, "winch");
        Lift lift = new Lift(telemetry, hardwareMap);
        BetterBoolGamepad bGamepad1 = new BetterBoolGamepad(gamepad1);
        BetterBoolGamepad bGamepad2 = new BetterBoolGamepad(gamepad2);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rClosed = false;
        lClosed = false;

        positionSet = false;
        interpolate = false;



        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y * speed,
                            -gamepad2.left_stick_x * speed,
                            -gamepad2.right_stick_x * speed
                    )
            );

            //right trigger to speed up, left trigger to slow down
            if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
            if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

            //slide.set(gamepad1.right_stick_y);
            double slideSpeed = gamepad2.right_bumper ? -0.25 : gamepad2.right_trigger;
            double winchSpeed = gamepad2.left_bumper ? 1 : -gamepad2.left_trigger;
            slide.setPower(slideSpeed);
            winch.setPower(winchSpeed);

            if(gamepad1.left_stick_y != 0)
            {
                telemetry.addData(" I", false);

                positionSet = false;
                interpolate = false;

                lift.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                lift.setLiftPower(gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_y * 0.5);

            }
            else {
                telemetry.addData(" I", true);

                if(gamepad1.right_stick_button)
                {
                    liftPos = 0;
                    lift.setWristPosFixed(0.9);
                }

                if(gamepad1.left_stick_button)
                {
                    liftPos = -1600;
                    lift.setWristPosFixed(0.38);
                }

                //lift.interpolateToEncoder(lift.liftL, lift.liftL.getTargetPosition(), 500, 5);
                //lift.interpolateToEncoder(lift.liftR, lift.liftR.getTargetPosition(), 500, 5);

            }

            telemetry.addData("Left Target", lift.liftL.getTargetPosition());
            telemetry.addData("Left", lift.liftL.getCurrentPosition());
            telemetry.addData("Right Target", lift.liftR.getTargetPosition());
            telemetry.addData("Right", lift.liftR.getCurrentPosition());

            telemetry.addData("Wrist Pos", lift.getWristPos());

            if(bGamepad1.right_bumper()) rClosed = !rClosed;
            if(bGamepad1.left_bumper()) lClosed = !lClosed;

            if(gamepad1.dpad_up) lift.setWristPos(drop3);
            if(gamepad1.dpad_right) lift.setWristPos(hover);
            if(gamepad1.dpad_down) lift.setWristPos(pickup);



            //if(gamepad1.y) lift.setLauncher(1);
            lift.calWrist(bGamepad2.dpad_up(), bGamepad2.dpad_down());

            lift.setRightClaw(rClosed);
            lift.setLeftClaw(lClosed);
            drive.update();

            lift.arm.moveTo(liftPos);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
