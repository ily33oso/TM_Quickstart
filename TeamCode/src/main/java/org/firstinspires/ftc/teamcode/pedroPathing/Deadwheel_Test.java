package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "DeadWheel_Test", group = "Test")
public class Deadwheel_Test extends LinearOpMode {

    DcMotor xEncoder; // side / strafe
    DcMotor yEncoder; // back / forward

    @Override
    public void runOpMode() {

        xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        yEncoder = hardwareMap.get(DcMotor.class, "yEncoder");

        // Reset encoders
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Use encoders without power
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("=== DEAD WHEEL TEST ===");
            telemetry.addData("X Encoder (Strafe)", xEncoder.getCurrentPosition());
            telemetry.addData("Y Encoder (Forward)", yEncoder.getCurrentPosition());

            telemetry.addLine("");
            telemetry.addLine("Push robot by hand:");
            telemetry.addLine("Forward  -> Y changes");
            telemetry.addLine("Strafe   -> X changes");
            telemetry.addLine("Rotate   -> minimal change");

            telemetry.update();
        }
    }
}
