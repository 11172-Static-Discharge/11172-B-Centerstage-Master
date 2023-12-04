package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name = "CuttleTeleLinear", group = "drive")
public class CuttleTeleLinear extends LinearOpMode {

    CalibrateLift calibrateLift;
    @Override
    public void runOpMode() {

        calibrateLift = new CalibrateLift(hardwareMap, telemetry);

        MechDrive drive = new MechDrive(hardwareMap, telemetry);


        double speed = 0.5;

        BetterBoolGamepad bGamepad2 = new BetterBoolGamepad(gamepad2);
        BetterBoolGamepad bGamepad1 = new BetterBoolGamepad(gamepad1);


        FirstBoolean lt = new FirstBoolean();
        FirstBoolean rt = new FirstBoolean();


        //calibrateLift.liftRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //calibrateLift.liftLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //starting lift at 0.5
        waitForStart();

        calibrateLift.liftLM.setTargetPosition(calibrateLift.liftLM.getCurrentPosition());
        calibrateLift.liftRM.setTargetPosition(calibrateLift.liftRM.getCurrentPosition());

        calibrateLift.liftLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        calibrateLift.liftRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        calibrateLift.liftLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        calibrateLift.liftRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        calibrateLift.liftRM.setPower();
        calibrateLift.liftLM.setVelocity(200);

        while (!isStopRequested()) {
            telemetry.addData("Lift Left Motor Target: ", calibrateLift.liftLM.getTargetPosition());
            telemetry.addData("Lift Left Motor Actual: ", calibrateLift.liftLM.getCurrentPosition());

            telemetry.addLine();

            telemetry.addData("Lift Right Motor Target: ", calibrateLift.liftRM.getTargetPosition());
            telemetry.addData("Lift Right Motor actual:", calibrateLift.liftRM.getCurrentPosition());

            telemetry.addData("wrist", calibrateLift.wrist.getPosition());
            telemetry.update();

            if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
            if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

            drive.drive(-gamepad1.left_stick_y * speed,
                    gamepad1.left_stick_x * speed,
                    gamepad1.right_stick_x * speed);

            //calibrateLift.calMotorLift(bGamepad2.dpad_up(), bGamepad2.dpad_down());
            calibrateLift.calWrist(bGamepad2.dpad_left(), bGamepad2.dpad_right());

            if (bGamepad1.a()) {
                calibrateLift.liftRM.setTargetPosition(100);
                calibrateLift.liftLM.setTargetPosition(-100);
            }

            calibrateLift.liftLM.setPower(0.5);
            calibrateLift.liftRM.pi
        }
    }
}