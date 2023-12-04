/*

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTeleCalibrate", group = "drive")
public class MotorTeleCalibrate extends OpMode {
    CalibrateLift calibrateLift;
    MechDrive drive;
    double speed;
    BetterBoolGamepad bGamepad2, bGamepad1;

    @Override
    public void init() {

        speed = 0.5;

        calibrateLift = new CalibrateLift(hardwareMap, telemetry);

        drive = new MechDrive(hardwareMap, telemetry);


        bGamepad2 = new BetterBoolGamepad(gamepad2);
        bGamepad1 = new BetterBoolGamepad(gamepad1);


        calibrateLift.liftLM.setTargetPosition(calibrateLift.liftLM.getCurrentPosition());
        calibrateLift.liftRM.setTargetPosition(calibrateLift.liftRM.getCurrentPosition());

        calibrateLift.liftLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        calibrateLift.liftRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    @Override
    public void loop() {
        telemetry.addData("Lift Left Motor predicted: ", calibrateLift.liftLMPos);
        telemetry.addData("Lift Left Motor Actual: ", calibrateLift.liftLM.getCurrentPosition());

        telemetry.addLine();

        telemetry.addData("Lift Right Motor predicted: ", calibrateLift.liftRMPos);
        telemetry.addData("Lift Right Motor actual:", calibrateLift.liftRM.getCurrentPosition());
        telemetry.update();

        if (gamepad1.right_trigger>0) speed = 0.5+gamepad1.right_trigger/2;
        if (gamepad1.left_trigger>0) speed = gamepad1.right_trigger/2;

        drive.drive(-gamepad1.left_stick_y * speed,
                gamepad1.left_stick_x * speed,
                gamepad1.right_stick_x * speed);

        calibrateLift.calMotorLift(bGamepad2.dpad_up(), bGamepad2.dpad_down());
    }
}*/