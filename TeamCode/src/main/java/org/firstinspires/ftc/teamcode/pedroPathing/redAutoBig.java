package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "red auto - big triangle - chasis")
public class redAutoBig extends LinearOpMode {

    // ---- DRIVE MOTORS ----
    DcMotor fl, fr, bl, br;

    // ---- MOTOR / ENCODER CONSTANTS ----
    // goBILDA 5203 Yellow Jacket, 312 RPM, 19.2:1
    // Encoder is on motor shaft: 537.7 ticks / motor rev
    // 537.7 * 19.2 = ~10362 ticks per output shaft revolution
    static final double TICKS_PER_REV = 10362.0;

    // CHANGE THIS TO MATCH YOUR WHEELS
    // 96mm mecanum = 3.78 inches
    static final double WHEEL_DIAMETER_IN = 4.09; // 104mm GripForce mecanum

    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- HARDWARE MAP ----
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // ---- MOTOR DIRECTIONS (typical mecanum) ----
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---- BRAKE WHEN POWER = 0 ----
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        telemetry.addLine("Blue Alliance Encoder Auto Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ---- DRIVE TO 24 INCHES FROM BASKET ----
// Example: if you start 60\" away, drive 36\"
            driveBackward(48, 0.5);
        }
    }

    // ================= HELPER METHODS =================

    private void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveBackward(double inches, double power) {
        int move = (int) (inches * TICKS_PER_INCH);

        // Left side must be negative because motors are mirrored
        setTarget(-move, move, -move, move);
        runToPosition(power);
    }




    private void setTarget(int flt, int frt, int blt, int brt) {
        fl.setTargetPosition(fl.getCurrentPosition() + flt);
        fr.setTargetPosition(fr.getCurrentPosition() + frt);
        bl.setTargetPosition(bl.getCurrentPosition() + blt);
        br.setTargetPosition(br.getCurrentPosition() + brt);
    }

    private void runToPosition(double power) {
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
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
        resetEncoders();
    }





    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
