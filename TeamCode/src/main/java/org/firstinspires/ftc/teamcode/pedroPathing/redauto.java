package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Red Auto", group = "Auto")
public class redauto extends LinearOpMode {

    DcMotor fl, fr, bl, br;

    static final double TICKS_PER_REV = 537.7; // goBILDA 5202/5203
    static final double WHEEL_DIAMETER = 3.77953; // mecanum (inches)
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Reverse left side
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {

            driveForward(24, 0.5);   // forward 24 inches
            strafeRight(18, 0.5);    // RED strafes RIGHT
            turnRight(12, 0.5);      // simple turn
        }
    }

    /* ------------------ MOVEMENT METHODS ------------------ */

    void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void driveForward(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        setTargets(ticks, ticks, ticks, ticks);
        setPower(power);
        waitForMotors();
    }

    void strafeRight(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        setTargets(ticks, -ticks, -ticks, ticks);
        setPower(power);
        waitForMotors();
    }

    void turnRight(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        setTargets(ticks, -ticks, ticks, -ticks);
        setPower(power);
        waitForMotors();
    }

    void setTargets(int flt, int frt, int blt, int brt) {
        fl.setTargetPosition(flt);
        fr.setTargetPosition(frt);
        bl.setTargetPosition(blt);
        br.setTargetPosition(brt);
    }

    void setPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    void waitForMotors() {
        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            idle();
        }
        setPower(0);
    }
}
