package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


@TeleOp(name = "SimpleMotorLift", group = "drive")
public class SimpleMotorLift extends LinearOpMode {
    DcMotorEx liftL;
    DcMotorEx liftR;

    @Override
    public void runOpMode() {
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");

        // Reset the encoder during initialization
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Set the motor's target position to 300 ticks
        liftL.setTargetPosition(liftL.getCurrentPosition());
        liftR.setTargetPosition(liftL.getCurrentPosition());

        // Switch to RUN_TO_POSITION mode
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        liftL.setVelocity(200);
        liftR.setVelocity(200);

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            telemetry.addData("velocity", liftL.getVelocity());
            telemetry.addData("position", liftL.getCurrentPosition());
            telemetry.addData("is at target", !liftL.isBusy());
            telemetry.update();
        }
    }
}