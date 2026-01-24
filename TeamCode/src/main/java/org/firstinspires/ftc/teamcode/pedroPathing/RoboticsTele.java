package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp( group = "Test")
public class RoboticsTele extends LinearOpMode {

    DcMotor intake;

    final int TICK_STEP = 90;
    final double FEED_POWER = 0.6;
    final double MOVE_POWER = 0.3;

    int targetPosition = 0;
    boolean lastRB = false;

    enum IntakeState {
        IDLE,
        MANUAL_FEED,
        MOVING_TO_TARGET
    }

    IntakeState intakeState = IntakeState.IDLE;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            boolean rb = gamepad2.right_bumper;

// X → manual feed
            if (gamepad2.x) {
                intakeState = IntakeState.MANUAL_FEED;
            }

// RB edge → add +90 ticks (SYNC FIRST!)
            else if (rb && !lastRB) {
                targetPosition = intake.getCurrentPosition(); // 🔑 CRITICAL FIX
                targetPosition += TICK_STEP;
                intakeState = IntakeState.MOVING_TO_TARGET;
            }

// No input
            else if (!gamepad2.x && !rb) {
                intakeState = IntakeState.IDLE;
            }

            lastRB = rb;


            // ---- State actions ----
            switch (intakeState) {

                case MANUAL_FEED:
                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intake.setPower(FEED_POWER);
                    break;

                case MOVING_TO_TARGET:
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setTargetPosition(targetPosition);
                    intake.setPower(MOVE_POWER);

                    // Stop applying power once target is reached
                    if (!intake.isBusy()) {
                        intake.setPower(0);
                        intakeState = IntakeState.IDLE;
                    }
                    break;

                case IDLE:
                    intake.setPower(0);
                    break;
            }

            telemetry.addData("State", intakeState);
            telemetry.addData("Current", intake.getCurrentPosition());
            telemetry.addData("Target", targetPosition);
            telemetry.update();
        }
    }
}
