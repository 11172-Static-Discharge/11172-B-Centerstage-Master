package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name = "Meet2TeleOpCalibrate", group = "drive")
public class Meet2TeleOpCalibrate extends LinearOpMode {

    public double[] hover = new double[]{0.1, 0.9, 0.9};
    public double[] pickup = new double[]{0.12, 0.88, 0.93};
    public double[] drop1 = new double[]{0.58, 0.42, 0.34};
    public double[] drop2 = new double[]{0.58, 0.42, 0.34};
    public double[] drop3 = new double[]{0.6, 0.4, 0.38};
    public double speed = 0.5;

    public double deadzone = 0.5;


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
        ColorRangeSensor sensorL = hardwareMap.get(ColorRangeSensor.class, "sensorL");
        ColorRangeSensor sensorR = hardwareMap.get(ColorRangeSensor.class, "sensorR");
        Lift lift = new Lift(telemetry, hardwareMap);
        BetterBoolGamepad bGamepad1 = new BetterBoolGamepad(gamepad1);
        BetterBoolGamepad bGamepad2 = new BetterBoolGamepad(gamepad2);

        lift.clawR.setPosition(0.5);
        lift.clawL.setPosition(0.5);

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




        waitForStart();

        while (!isStopRequested()) {
            if(gamepad2.right_bumper)
            {
                double speed = 0.35;
                drive.setMotorPowers(speed, -speed, speed, -speed);
            }
            else if(gamepad2.left_bumper)
            {
                double speed = 0.35;
                drive.setMotorPowers(-speed, speed, -speed, speed);
            }
            else
                drive.setWeightedDrivePower(
                        new Pose2d(
                                Math.abs(gamepad2.left_stick_y) > deadzone ? -gamepad2.left_stick_y * speed : 0,
                                Math.abs(gamepad2.left_stick_x) > deadzone ?-gamepad2.left_stick_x * speed : 0,
                                Math.abs(gamepad2.right_stick_x) > deadzone ?-gamepad2.right_stick_x * speed : 0
                        )
                );


            if(gamepad2.share) requestOpModeStop();
            //if(gamepad2.square) lift.setLauncher(0);

            //right trigger to speed up, left trigger to slow down
            if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
            if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

            if(autoClose)
            {
                double slideSpeed = gamepad1.right_bumper ? -0.5 : gamepad1.right_trigger;
                double winchSpeed = gamepad1.left_bumper ? 1 : -gamepad1.left_trigger;
                slide.setPower(slideSpeed);
            }

            else {
                if(bGamepad1.right_bumper()) rClosed = !rClosed;
                if(bGamepad1.left_bumper()) lClosed = !lClosed;
            }
            //slide.set(gamepad1.right_stick_y);


            if(Math.abs(gamepad1.left_stick_y) >= 0.2)
            {
                telemetry.addData(" I", false);

                positionSet = false;
                interpolate = false;

                lift.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                lift.setLiftPower(gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_y * 0.5);

                liftPos = lift.liftL.getCurrentPosition();

            }
            else {

                if(gamepad1.right_stick_button)
                {
                    liftPos = -35;
                    lift.setWristPosFixed(0.85);
                }

                if(gamepad1.left_stick_button)
                {
                    liftPos = -1320;
                    lift.setWristPosFixed(0.22);
                }

                if(gamepad1.dpad_left)
                {
                    liftPos = -1700;
                    lift.setWristPosFixed(0.37);
                }

                //lift.interpolateToEncoder(lift.liftL, lift.liftL.getTargetPosition(), 500, 5);
                //lift.interpolateToEncoder(lift.liftR, lift.liftR.getTargetPosition(), 500, 5);
            }
            lift.arm.moveTo(liftPos);
            if(gamepad1.square) linRegMode = !linRegMode;



            telemetry.addData("Left Target", lift.liftL.getTargetPosition());
            telemetry.addData("Left", lift.liftL.getCurrentPosition());
            telemetry.addData("Right Target", lift.liftR.getTargetPosition());
            telemetry.addData("Right", lift.liftR.getCurrentPosition());
            //telemetry.addData("Distance sensor L: ", sensorL.getDistance(DistanceUnit.INCH));
            //telemetry.addData("Distance sesnor R: ", sensorR.getDistance(DistanceUnit.INCH));
            //telemetry.addData("Close mode auto:", autoClose);
            //telemetry.addData("Auto closed L", lift.autoClosedL);
            //telemetry.addData("autoclosed r: ", lift.autoClosedR);
            telemetry.addData("Wrist Pos", lift.getWristPos());



            if (bGamepad1.b()) autoClose = !autoClose;
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

            if(linRegMode) lift.setWristPosFixed(WristReg(lift.liftL.getCurrentPosition(), offset));


            if(gamepad1.dpad_up) lift.setWristPosFixed(0.25);
            if(gamepad1.dpad_right) lift.setWristPos(hover);
            if(gamepad1.dpad_down) lift.setWristPos(pickup);



            //if(gamepad1.y) lift.setLauncher(1);


            //lift.setRightClaw(rClosed);
            //lift.setLeftClaw(lClosed);
            drive.update();

            lift.calWrist(bGamepad2.dpad_up(), bGamepad2.dpad_down());
            //lift.calDispenser(bGamepad2.y(), bGamepad2.a());
            lift.calClaw(bGamepad2.b(), bGamepad2.a(), bGamepad2.x(), bGamepad2.y());

            telemetry.addData("clawRPosMain: ", lift.clawR.getPosition());
            telemetry.addData("clawLPosMain: ", lift.clawL.getPosition());

            Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public double WristReg(int liftPos, double offset)
    {
        return (-3.947 * liftPos ) - 0.301 + offset;
    }
}
