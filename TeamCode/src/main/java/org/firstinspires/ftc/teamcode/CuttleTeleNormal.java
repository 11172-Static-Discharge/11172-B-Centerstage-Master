package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled

@TeleOp(name = "CuttleTeleNormal", group = "drive")
public class CuttleTeleNormal extends OpMode {

    //lift wrapper
    CalibrateLift calibrateLift;

    //drivetrain, can be anything
    MechDrive drive;


    //percent speed the bot drives at
    double speed;

    //idk

    //gamepad that registers a hold of a button as one press
    BetterBoolGamepad bGamepad2, bGamepad1;

    FirstBoolean lt;
    FirstBoolean rt;


    @Override
    public void init() {

        //50% speed
        speed = 0.5;

        //DcMotor slide = hardwareMap.dcMotor.get("slide");

        //Servo launcher = hardwareMap.servo.get("launcher");


        //init lift wrapper
        calibrateLift = new CalibrateLift(hardwareMap, telemetry);

        //init drive
        drive = new MechDrive(hardwareMap, telemetry);

        //init better gamepad
        bGamepad2 = new BetterBoolGamepad(gamepad2);
        bGamepad1 = new BetterBoolGamepad(gamepad1);

        lt = new FirstBoolean();
        rt = new FirstBoolean();


        calibrateLift.clawL.setPosition(calibrateLift.clawLClose);
        calibrateLift.clawR.setPosition(calibrateLift.clawRClose);

    }
    @Override
    public void loop() {

        //telemety
        telemetry.addData("actual wrist: ", calibrateLift.wrist.getPosition());

        telemetry.addLine();

        telemetry.addData("actual liftL: ", calibrateLift.liftLM.getCurrentPosition());
        telemetry.addData("actual liftR: ", calibrateLift.liftRM.getCurrentPosition());

        telemetry.addLine();

        telemetry.addData("clawL: ", calibrateLift.clawL.getPosition());
        telemetry.addData("clawR: ", calibrateLift.clawR.getPosition());


        telemetry.addLine();

        telemetry.addData("expected wristPos: ", calibrateLift.wristPos);

        telemetry.addLine();


        telemetry.addData("expected clawLPos", calibrateLift.clawLPos);
        telemetry.addData("actual clawL", calibrateLift.clawL.getPosition());

        telemetry.addLine();

        telemetry.addData("expected clawRPos", calibrateLift.clawR.getPosition());
        telemetry.addData("actual clawR", calibrateLift.clawR.getPosition());

        telemetry.update();


        //right trigger to speed up, left trigger to slow down
        if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
        if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

        //drive
        drive.drive(-gamepad1.left_stick_y * speed,
                gamepad1.left_stick_x * speed,
                gamepad1.right_stick_x * speed);

        //using claw
        calibrateLift.useClaw(bGamepad2.left_bumper(), lt.betterboolean(gamepad2.left_trigger>0) , bGamepad2.right_bumper(), rt.betterboolean(gamepad2.right_trigger>0));





        //if (bGamepad2.a()) calibrateLift.setLiftPos(0);
        //if (bGamepad2.x() || bGamepad2.b()) lift.setLiftPos(1);
        //if (bGamepad2.y()) lift.setLiftPos(2);

        //lift.toggleAutoClose(bGamepad2.b());





        calibrateLift.calWrist(bGamepad2.dpad_left(), bGamepad2.dpad_right());

        calibrateLift.calClaw(bGamepad1.dpad_up(), bGamepad1.dpad_down(), bGamepad1.dpad_left(), bGamepad1.dpad_right());

        //setting lift pos
        //if (gamepad2.x) lift.setLiftPos(0);
        //if (gamepad2.square || gamepad2.circle) lift.setLiftPos(1);
        //if (gamepad2.triangle) lift.setLiftPos(2);
        //lift.calservo(bGamepad1.dpad_up(), bGamepad1.dpad_down(), bGamepad1.dpad_left(), bGamepad1.dpad_right());

        //slide
        //slide.setPower(gamepad2.right_stick_y);

        //paper launcher
        //if (gamepad2.right_trigger>0.1) launcher.setPosition(1);
        //if (gamepad2.left_trigger>0.1) launcher.setPosition(0);
        }
    }