package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel_PID_Test", group = "Test")
public class Flywheel_PID_Test extends LinearOpMode {

    private DcMotorEx rwheel;
    private DcMotorEx lwheel;
    DcMotor intake;

    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 100;// from far 2250

    private double targetVelocity;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // ✅ RunMode comes from DcMotor
        rwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        telemetry.addLine("Flywheel PID Test Ready");
        telemetry.addLine("A = Spin Up | B = Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);
            }

            if (gamepad1.b) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }

            if (gamepad1.x){
                intake.setPower(-.75);
            } else {
                intake.setPower(0);
            }

            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM  = (lwheel.getVelocity() * 60) / TICKS_PER_REV;

            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Right RPM", rightRPM);
            telemetry.addData("Left RPM", leftRPM);
            telemetry.update();
        }
    }
}
