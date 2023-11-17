package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "CuttleTele", group = "drive")
public class CuttleTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.5;

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        Servo launcher = hardwareMap.servo.get("launcher");

        CuttleLift lift = new CuttleLift(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            //right trigger to speed up, left trigger to slow down
            if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
            if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

            //drive
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speed,
                            -gamepad1.left_stick_x * speed,
                            -gamepad1.right_stick_x * speed
                    )
            );

            //using claw
            lift.useClaw(gamepad2.left_bumper, gamepad2.right_bumper);

            //setting lift pos
            if (gamepad2.x) lift.setLiftPos(0);
            if (gamepad2.square || gamepad2.circle) lift.setLiftPos(1);
            if (gamepad2.triangle) lift.setLiftPos(2);

            //slide
            slide.setPower(gamepad2.right_stick_y);

            //paper launcher
            if (gamepad2.right_trigger>0.1) launcher.setPosition(1);
            if (gamepad2.left_trigger>0.1) launcher.setPosition(0);
        }
    }
}