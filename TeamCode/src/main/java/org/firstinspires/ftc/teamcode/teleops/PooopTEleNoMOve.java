package org.firstinspires.ftc.teamcode.teleops;

import android.speech.RecognitionService;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name = "PoopTeleNoMove", group = "drive")
public class PooopTEleNoMOve extends OpMode {

    public PIDController lcontroller, rcontroller;
    public static double p=0.1,i=0,d=0.0001,f=0.22;

    public double speed = 0.65;

    public DcMotorEx l,r,slide,winch;

    public final double ticksInDegree = 288 / 360.0;

    public static int targetL = 0;

    public SampleMecanumDrive drive;

    public Lift lift;
    public BetterBoolGamepad bGamepad1, bGamepad2;

    public double deadzone = 0.5;


    public double offset;

    public boolean linRegMode;


    public boolean rClosed, lClosed;
    public boolean positionSet, interpolate;

    public int liftPos;
    public boolean autoClose;
    public double wristPos;


    public void init() {
        lcontroller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        l = hardwareMap.get(DcMotorEx.class, "liftL");
        r = hardwareMap.get(DcMotorEx.class, "liftR");

        lift = new Lift(telemetry, hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);

        slide = hardwareMap.get(DcMotorEx.class, "slide");

        bGamepad1 = new BetterBoolGamepad(gamepad1);
        bGamepad2 = new BetterBoolGamepad(gamepad2);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setDispenser(1);


        /*l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        rClosed = true;
        autoClose = false;
        lClosed = true;

        linRegMode = false;
        offset = 0;

        positionSet = false;
        interpolate = false;
    }
    public void loop() {
        //drive.setWeightedDrivePower(
          ///      new Pose2d(
             //           Math.abs(gamepad2.left_stick_y) > deadzone ? -gamepad2.left_stick_y * speed : 0,
               //         Math.abs(gamepad2.left_stick_x) > deadzone ?-gamepad2.left_stick_x * speed : 0,
                 //       Math.abs(gamepad2.right_stick_x) > deadzone ?-gamepad2.right_stick_x * speed : 0
                //)
        //);

        if(gamepad2.share) requestOpModeStop();
        if(gamepad2.square) lift.setLauncher(0);

        double slideSpeed = gamepad2.right_bumper ? 1 : -gamepad2.right_trigger;
        slide.setPower(slideSpeed);

        /*
        if(slide.getPosition()><0 || power<0) {
            slide.setPower(slideSpeed);
        }
        */

        if(bGamepad1.right_bumper()) rClosed = !rClosed;
        if(bGamepad1.left_bumper()) lClosed = !lClosed;

        lift.setRightClaw(rClosed);
        lift.setLeftClaw(lClosed);


        telemetry.addData("Wrist Pos", lift.getWristPos());
        telemetry.addData("Lift Pos", l.getCurrentPosition());
        telemetry.addData("Target Lift Pos", targetL);
        telemetry.addData("Slide Pos", slide.getCurrentPosition());


        /*if(bGamepad1.y())
        {
            lift.setWristPosFixed(lift.getWristPos() - 0.02);
            offset -= 0.02;
        }
        if(bGamepad1.a())
        {
            lift.setWristPosFixed(lift.getWristPos() + 0.02);
            offset += 0.02;
        }*/

        if(gamepad1.square) lift.setWristPosFixed(0.25);
        if(gamepad1.share) lift.setDispenser(1);

        //LIFT STUFF HERE
        if(Math.abs(gamepad1.left_stick_y) >= 0.2)
        {
            l.setPower(gamepad1.left_stick_y * (gamepad1.share ? 1 : 0.5));
            r.setPower(gamepad1.left_stick_y * (gamepad1.share ? 1 : 0.5));
            targetL = l.getCurrentPosition();
            return;
        }


        if(gamepad1.dpad_left)
        {
            targetL = -130;
            lift.setWristPosFixed(0.28);
        }

        if(gamepad1.dpad_up)
        {
            targetL = -115;
            lift.setWristPosFixed(0.22);
        }

        if(gamepad1.dpad_right)
        {
            targetL = -20;
            lift.setWristPosFixed(0.25);
        }

        if(gamepad1.dpad_down)
        {
            targetL = -13;
            lift.setWristPosFixed(0.7625);
        }

        lcontroller.setPID(p,i,d);
        int lPos = l.getCurrentPosition();
        if(lPos > 0) return;

        double pid = lcontroller.calculate(lPos, targetL);

        double ff = Math.cos(Math.toRadians(targetL / ticksInDegree)) * f;


        double power = pid + ff;

        l.setPower(power);
        r.setPower(power);
        telemetry.addData("lPos: ", lPos);
        telemetry.addData("lTarget: ", targetL);
        telemetry.update();
    }
}
