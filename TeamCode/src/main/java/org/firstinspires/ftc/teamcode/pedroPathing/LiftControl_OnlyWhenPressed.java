package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="LiftControl_OnlyWhenPressed")
public class LiftControl_OnlyWhenPressed extends LinearOpMode {

    // ---- CONSTANTS FOR 5203 (435 RPM) + 8MM LEADSCREW ----
    final double TICKS_PER_MM = 48.0625;             // 384.5 PPR / 8mm pitch
    final double MAX_MM = 291.28738622;                     // 12 inches
    final int MAX_TICKS = (int)(TICKS_PER_MM * MAX_MM); // = 14649
    final int MIN_TICKS = 0;

    final double UP_POWER   = 1.0;   // speed going up
    final double DOWN_POWER = -1.0;  // speed going down

    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100); // 50–200ms usually enough
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            int pos = lift.getCurrentPosition();

            // ----- DPAD UP: move up -----
            if (gamepad1.dpad_up) {
                if (pos < MAX_TICKS) {
                    lift.setPower(UP_POWER);
                } else {
                    lift.setPower(0);
                }

                // ----- DPAD DOWN: move down -----
            } else if (gamepad1.dpad_down) {
                if (pos > MIN_TICKS) {
                    lift.setPower(DOWN_POWER);
                } else {
                    lift.setPower(0);
                }

                // ----- NOTHING PRESSED: do nothing -----
            } else {
                lift.setPower(0);   // completely idle
            }

            telemetry.addData("Lift Pos", pos);
            telemetry.addData("Min-Max", "%d / %d", MIN_TICKS, MAX_TICKS);
            telemetry.update();
        }
    }
}
