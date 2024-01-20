package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.ServoCalibrator;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "LiftTele", group = "drive")
public class LiftTele extends LinearOpMode {

    public double[] hover = new double[]{0.1, 0.9, 0.9};
    public double[] pickup = new double[]{0.12, 0.88, 0.93};
    public double[] drop1 = new double[]{0.58, 0.42, 0.34};
    public double[] drop2 = new double[]{0.58, 0.42, 0.34};
    public double[] drop3 = new double[]{0.6, 0.4, 0.38};
    public double speed = 0.5;

    public double deadzone = 0.5;
    public double powMultiplier = 1;


    public double offset;

    public boolean linRegMode;


    public boolean rClosed, lClosed;
    public boolean positionSet, interpolate;

    public int liftPos;
    public boolean autoClose;
    public double wristPos;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Slide slide = new Slide(telemetry, hardwareMap);

        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slide");
        DcMotorEx winch = hardwareMap.get(DcMotorEx.class, "winch");
        ColorRangeSensor sensorL = hardwareMap.get(ColorRangeSensor.class, "sensorL");
        ColorRangeSensor sensorR = hardwareMap.get(ColorRangeSensor.class, "sensorR");
        Lift lift = new Lift(telemetry, hardwareMap);
        BetterBoolGamepad bGamepad1 = new BetterBoolGamepad(gamepad1);
        BetterBoolGamepad bGamepad2 = new BetterBoolGamepad(gamepad2);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rClosed = true;
        autoClose = false;
        lClosed = true;

        linRegMode = false;
        offset = 0;

        positionSet = false;
        interpolate = false;

        liftPos = -15;



        waitForStart();

        while (!isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.abs(gamepad2.left_stick_y) > deadzone ? -gamepad2.left_stick_y * speed : 0,
                            Math.abs(gamepad2.left_stick_x) > deadzone ?-gamepad2.left_stick_x * speed : 0,
                            Math.abs(gamepad2.right_stick_x) > deadzone ?-gamepad2.right_stick_x * speed : 0
                    )
            );


            if(gamepad2.share) requestOpModeStop();
            if(gamepad2.square) lift.setLauncher(0);

            if(autoClose)
            {
                double slideSpeed = gamepad1.right_bumper ? -0.5 : gamepad1.right_trigger;
                double winchSpeed = gamepad1.left_bumper ? 1 : -gamepad1.left_trigger;
                slide.setPower(slideSpeed);
                winch.setPower(winchSpeed);
            }

            else {
                if(bGamepad1.right_bumper()) rClosed = !rClosed;
                if(bGamepad1.left_bumper()) lClosed = !lClosed;
            }
            //slide.set(gamepad1.right_stick_y);


            if(Math.abs(gamepad1.left_stick_y) >= 0.2)
            {
                lift.arm.setPower(gamepad1.left_stick_y);
            }
            else {
                if(gamepad1.dpad_up)
                {
                    liftPos = -120;
                    lift.setWristPosFixed(0.22);
                }

                if(gamepad1.dpad_right)
                {
                    liftPos = -20;
                    lift.setWristPosFixed(0.25);
                }

                if(gamepad1.dpad_down)
                {
                    liftPos = -13;
                    lift.setWristPosFixed(0.85);
                }
                lift.moveToPower(liftPos, powMultiplier);
            }


            telemetry.addData("Left Target", liftPos);
            telemetry.addData("Wrist Pos", lift.getWristPos());



            if(bGamepad1.y())
            {
                lift.setWristPosFixed(lift.getWristPos() - 0.02);
                offset -= 0.02;
            }
            if(bGamepad1.a())
            {
                lift.setWristPosFixed(lift.getWristPos() + 0.02);
                offset += 0.02;
            }

            if(gamepad1.circle) autoClose = !autoClose;
            if(gamepad1.dpad_left) lift.setWristPosFixed(0.25);


            lift.setRightClaw(rClosed);
            lift.setLeftClaw(lClosed);
            drive.update();

            //lift.calWrist(bGamepad2.dpad_up(), bGamepad2.dpad_down());


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
