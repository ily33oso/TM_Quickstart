package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleyteley extends LinearOpMode {

    // ---------------- CHASSIS ----------------
    DcMotor fr, fl, bl, br;

    // ---------------- MECHANISMS ----------------
    DcMotor lwheel, rwheel, intake, lift;
    Servo rstopper;

    // ---------------- IMU ----------------
    IMU imu;

    // ---------------- INTAKE TOGGLE ----------------
    boolean intakeOn = false;
    boolean intakeOpposite = true;
    boolean xWasPressed = false;
    boolean yWasPressed = false;

    // ---------------- LIFT CONSTANTS ----------------
    final double TICKS_PER_MM = 48.0625;
    final double MAX_MM = 291.28738622;
    final int MAX_TICKS = (int)(TICKS_PER_MM * MAX_MM);
    final int MIN_TICKS = 0;

    final double UP_POWER = 1.0;
    final double DOWN_POWER = -75.0;

    @Override
    public void runOpMode() {

        // ----------- HARDWARE MAP -----------
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");

        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");
        lift   = hardwareMap.dcMotor.get("lift");

        rstopper = hardwareMap.servo.get("rstopper");

        // ----------- DIRECTIONS -----------
        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rstopper.setDirection(Servo.Direction.FORWARD);

        // ----------- LIFT ENCODER INIT -----------
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ----------- IMU INIT -----------
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        // ----------- MAIN LOOP -----------
        while (opModeIsActive()) {

            // ---------- DRIVETRAIN ----------
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = lx * Math.cos(heading) - ly * Math.sin(heading);
            double rotatedY = lx * Math.sin(heading) + ly * Math.cos(heading);

            double max = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1);
            double drivePower = 0.9 - (0.6 * gamepad1.right_trigger);

            br.setPower(((rotatedY - rotatedX - rx) / max) * drivePower);
            bl.setPower(((rotatedY + rotatedX + rx) / max) * drivePower);
            fr.setPower(((rotatedY + rotatedX - rx) / max) * drivePower);
            fl.setPower(((rotatedY - rotatedX + rx) / max) * drivePower);

            if (gamepad1.y) imu.resetYaw();

            // ---------- SHOOTER ----------
            if (gamepad2.a) {
                rwheel.setPower(0.78);
                lwheel.setPower(0.78);
            } else {
                rwheel.setPower(0);
                lwheel.setPower(0);
            }

            // ---------- INTAKE TOGGLE ----------
            if (gamepad2.x && !xWasPressed) {
                intakeOn = !intakeOn;
                sleep(120);
            }
            xWasPressed = gamepad2.x;

            if (gamepad2.y && !yWasPressed) {
                intakeOpposite = !intakeOpposite;
                sleep(120);
            }
            yWasPressed = gamepad2.y;

            if (intakeOn) {
                intake.setPower(intakeOpposite ? 0.5 : -0.75);
            } else {
                intake.setPower(0);
            }

            // ---------- STOPPER ----------
            rstopper.setPosition(gamepad2.b ? 0.57 : 0);

            // ---------- LIFT CONTROL (ONLY WHEN PRESSED) ----------
            int pos = lift.getCurrentPosition();

            if (gamepad1.dpad_up && pos < MAX_TICKS) {
                lift.setPower(UP_POWER);
            } else if (gamepad1.dpad_down && pos > MIN_TICKS) {
                lift.setPower(DOWN_POWER);
            } else {
                lift.setPower(0);
            }

            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.addData("Lift Pos", pos);
            telemetry.addData("Lift Range", "%d / %d", MIN_TICKS, MAX_TICKS);
            telemetry.update();
        }
    }
}
