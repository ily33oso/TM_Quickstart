package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "turning")
public class turning_tuning extends LinearOpMode {
    DcMotor fl, fr, bl, br;

    // CONSTANTS FOR YOUR ROBOT
    static final double TICKS_PER_REV = 537.7;  // goBILDA 312 RPM
    static final double WHEEL_DIAMETER_IN = 4.0;
    static final double ROBOT_TURN_CIRCUMFERENCE = 40.0;  // tuned for 15.5" wheelbase

    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    static final double TICKS_PER_DEGREE =
            (ROBOT_TURN_CIRCUMFERENCE * TICKS_PER_INCH) / 360.0;

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        // LEFT SIDE REVERSED (as you said)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("READY FOR START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --------------------------------------
        // 1) DRIVE STRAIGHT (48 in as you chose)
        // --------------------------------------
        encoderDrive(48, 0.5);

        telemetry.addLine("✓ Finished driving straight (48 in)");
        telemetry.addData("Time", getRuntime());
        telemetry.update();
        sleep(300);

        // --------------------------------------
        // 2) TURN LEFT TO FACE DEPOT (90°)
        // --------------------------------------
        turnDegrees(90, 0.5);

        telemetry.addLine("✓ Finished turning LEFT 90° (Facing Depot)");
        telemetry.addData("Time", getRuntime());
        telemetry.update();
    }


    // -----------------------------------------------------
    // ENCODER DRIVE FUNCTION
    // -----------------------------------------------------
    public void encoderDrive(double inches, double power) {

        int ticks = (int)(inches * TICKS_PER_INCH);

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            telemetry.addData("Driving", "%d ticks", ticks);
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // -----------------------------------------------------
    // ENCODER TURN FUNCTION
    // -----------------------------------------------------
    public void turnDegrees(double degrees, double power) {

        int ticks = (int)(degrees * TICKS_PER_DEGREE);

        // LEFT TURN → left motors forward, right motors backward
        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            telemetry.addData("Turning Left", "%d ticks", ticks);
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // -----------------------------------------------------
    // STOP FUNCTION
    // -----------------------------------------------------
    public void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
