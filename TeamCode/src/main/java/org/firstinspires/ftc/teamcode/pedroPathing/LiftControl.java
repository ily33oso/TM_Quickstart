package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LiftControl")
public class LiftControl extends LinearOpMode {

    // ---- HARDWARE-SPECIFIC CONSTANTS ----
    final double TICKS_PER_MM = 48.0625;     // 384.5 PPR / 8 mm pitch
    final double MAX_MM = 304.8;            // 12 inches
    final int MAX_TICKS = (int) Math.round(TICKS_PER_MM * MAX_MM); // = 14649
    final int MIN_TICKS = 0;

    // speeds / tune these as needed
    final double UP_POWER   = -1.0;          // full up speed (tune down if too fast)
    final double DOWN_POWER = 1.0;         // down speed (negative)
    final double HOLD_POWER = 0.12;         // small holding power — tune for your load

    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {

        lift = hardwareMap.get(DcMotor.class, "lift");

        // safety / behavior
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("MaxTicks", MAX_TICKS);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int pos = lift.getCurrentPosition();

            // ---- DPAD UP: raise ----
            if (gamepad1.dpad_up) {
                if (pos < MAX_TICKS) {
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(UP_POWER);
                } else {
                    // reached top
                    lift.setPower(0);
                }

                // ---- DPAD DOWN: lower ----
            } else if (gamepad1.dpad_down) {
                if (pos > MIN_TICKS) {
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(DOWN_POWER);
                } else {
                    // reached bottom
                    lift.setPower(0);
                }

                // ---- else: hold ----
            } else {
                // small hold power prevents drop; switch to RUN_TO_POSITION hold if preferred
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(HOLD_POWER);
            }

            telemetry.addData("Pos", pos);
            telemetry.addData("TicksPerMM", TICKS_PER_MM);
            telemetry.addData("MaxTicks", MAX_TICKS);
            telemetry.update();
        }
    }
}
