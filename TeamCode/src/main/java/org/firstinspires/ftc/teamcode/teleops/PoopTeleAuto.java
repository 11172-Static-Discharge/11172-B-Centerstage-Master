package org.firstinspires.ftc.teamcode.teleops;

import android.speech.RecognitionService;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.config.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name = "PoopTeleAuto", group = "drive")
public class PoopTeleAuto extends OpMode {

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

    public double powMultiplier = 1;
    public Pose2d poseEstimate;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode;
    int lPos;
    double targetAngle = Math.toRadians(90);


    public void init() {


        lcontroller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        l = hardwareMap.get(DcMotorEx.class, "liftL");
        r = hardwareMap.get(DcMotorEx.class, "liftR");

        lift = new Lift(telemetry, hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        currentMode = Mode.DRIVER_CONTROL;

        slide = hardwareMap.get(DcMotorEx.class, "slide");

        bGamepad1 = new BetterBoolGamepad(gamepad1);
        bGamepad2 = new BetterBoolGamepad(gamepad2);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift.setDispenser(1);

        poseEstimate = new Pose2d();


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
        poseEstimate = drive.getPoseEstimate();
        drive.update();
        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                Math.abs(gamepad2.left_stick_y) > deadzone ? -gamepad2.left_stick_y * speed : 0,
                                Math.abs(gamepad2.left_stick_x) > deadzone ? -gamepad2.left_stick_x * speed : 0,
                                Math.abs(gamepad2.right_stick_x) > deadzone ? -gamepad2.right_stick_x * speed : 0
                        )
                );
                if(gamepad2.x) {
                    drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                if (gamepad2.b) {
                    currentMode = Mode.DRIVER_CONTROL;
                }

                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }
        if(gamepad2.share) requestOpModeStop();
        if(gamepad2.y) lift.setLauncher(1);

        if(bGamepad2.a()) speed = (speed == 0.65) ? 0.29 : 0.65;
        if(bGamepad1.a()) powMultiplier = (powMultiplier == 1) ? 0.2 : 1;


        double slideSpeed = gamepad2.right_bumper ? -1 : gamepad2.right_trigger;

        if(slide.getCurrentPosition()<0 || slideSpeed<0) {
            slide.setPower(slideSpeed);
        }else
        {
            slide.setPower(0);
        }

        if(bGamepad1.right_bumper()) rClosed = !rClosed;
        if(bGamepad1.left_bumper()) lClosed = !lClosed;

        lift.setRightClaw(rClosed);
        lift.setLeftClaw(lClosed);

        //speed = (targetL<lPos) ? 0.65 : 0.29;

        //lift.calWrist(bGamepad2.dpad_up(), bGamepad2.dpad_down());
        //lift.calDispenser(bGamepad2.dpad_left(), bGamepad1.dpad_right());

        telemetry.addData("Wrist Pos", lift.getWristPos());
        telemetry.addData("Lift Pos", l.getCurrentPosition());
        telemetry.addData("Target Lift Pos", targetL);
        telemetry.addData("Slide Pos", slide.getCurrentPosition());
        telemetry.addData("Lin Reg Mode", linRegMode);
        telemetry.addData("Pigga Multipligga", powMultiplier);
        telemetry.addData("mode", currentMode);
        telemetry.addData("heading", poseEstimate.getHeading());



        if(bGamepad1.y())
        {
            lift.setWristPosFixed(lift.getWristPos() - 0.02);
            offset -= 0.02;
        }
        if(bGamepad1.b()) {
            lift.setWristPosFixed(lift.getWristPos() + 0.02);
            offset += 0.02;
        }



        if(gamepad1.square)
        {
            liftPos = 5;
            lift.setWristPosFixed(Lift.wristHover);
        }
        if(gamepad1.share) lift.setDispenser(1);

        //LIFT STUFF HERE
        if(Math.abs(gamepad1.left_stick_y) >= 0.2)
        {
            l.setPower(gamepad1.left_stick_y * (gamepad1.share ? (1) : 0.5));
            r.setPower(gamepad1.left_stick_y * (gamepad1.share ? (1) : 0.5));
            targetL = l.getCurrentPosition();
            return;
        }


        if(gamepad1.dpad_left)
        {
            targetL = -130;
            lift.setWristPosFixed(Lift.wristMid);
        }

        if(gamepad1.dpad_up)
        {
            targetL = -115;
            lift.setWristPosFixed(Lift.wristHigh);
        }

        if(gamepad1.dpad_right)
        {
            targetL = -40;
            lift.setWristPosFixed(0);
        }

        /*if(gamepad1.dpad_down) {
            targetL = 5;
            lift.setWristPosFixed((lPos-targetL < -100 ? 0: Lift.wristHover);
        }*/

        if(gamepad1.dpad_down)
        {
            targetL = 5;
            lift.setWristPosFixed(Lift.wristPickup);
        }

        lcontroller.setPID((p * powMultiplier),i,d);

        lPos = l.getCurrentPosition();

        if(lPos > 0 || ((liftPos == 5 || liftPos == -20) && Math.abs(lPos - targetL) < 5))
        {
            l.setPower(0);
            r.setPower(0);
            telemetry.addData("PEEEEPEE", true);
            telemetry.update();
            return;
        }




        double pid = lcontroller.calculate(lPos, targetL);

        double ff = Math.cos(Math.toRadians(targetL / ticksInDegree)) * f;


        double power = pid + ff;

        l.setPower(power);
        r.setPower(power);
        telemetry.addData("lPos: ", lPos);
        telemetry.addData("lTarget: ", targetL);
        telemetry.update();
    }

    public double wristReg(int liftPos){return (double)liftPos * -0.008 - 0.85;}
}
