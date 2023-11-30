package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CuttleTeleNormal", group = "drive")
public class CuttleTeleNormal extends OpMode {

    //lift wrapper
    CuttleLift lift;

    //drivetrain, can be anything
    MechDrive drive;


    //percent speed the bot drives at
    double speed;

    //idk

    //gamepad that registers a hold of a button as one press
    BetterBoolGamepad bGamepad2;

    FirstBoolean lt;
    FirstBoolean rt;


    @Override
    public void init() {

        //50% speed
        speed = 0.5;

        //DcMotor slide = hardwareMap.dcMotor.get("slide");

        //Servo launcher = hardwareMap.servo.get("launcher");


        //init lift wrapper
        lift = new CuttleLift(hardwareMap, telemetry);

        //init drive
        drive = new MechDrive(hardwareMap, telemetry);

        //init better gamepad
        bGamepad2 = new BetterBoolGamepad(gamepad2);

        lt = new FirstBoolean();
        rt = new FirstBoolean();

        //starting lift at 0.5
        lift.liftL.setPosition(0.5);
        lift.liftR.setPosition(0.5);

    }
    @Override
    public void loop() {

        //telemety
        telemetry.addData("wrist: ", lift.wrist.getPosition());

        telemetry.addData("liftL: ", lift.liftL.getPosition());
        telemetry.addData("liftR: ", lift.liftR.getPosition());

        telemetry.addData("clawL: ", lift.clawL.getPosition());
        telemetry.addData("clawR: ", lift.clawR.getPosition());

        telemetry.addData("liftLPos: ", lift.liftLPos);
        telemetry.addData("liftRPos: ", lift.liftRPos);
        telemetry.addData("wristPos: ", lift.wristPos);

        telemetry.addData("collection mode: ", lift.closeMode);
        telemetry.update();


        //right trigger to speed up, left trigger to slow down
        if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
        if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

        //drive
        drive.drive(-gamepad1.left_stick_y * speed,
                gamepad1.left_stick_x * speed,
                gamepad1.right_stick_x * speed);

        //using claw
        lift.useClaw(bGamepad2.left_bumper(), lt.betterboolean(gamepad2.left_trigger>0) , bGamepad2.right_bumper(), rt.betterboolean(gamepad2.right_trigger>0));





        if (bGamepad2.a()) lift.setLiftPos(0);
        //if (bGamepad2.x() || bGamepad2.b()) lift.setLiftPos(1);
        //if (bGamepad2.y()) lift.setLiftPos(2);

        //lift.toggleAutoClose(bGamepad2.b());





        lift.calLift(bGamepad2.dpad_up(), bGamepad2.dpad_down());

        lift.calWrist(bGamepad2.dpad_left(), bGamepad2.dpad_right());

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