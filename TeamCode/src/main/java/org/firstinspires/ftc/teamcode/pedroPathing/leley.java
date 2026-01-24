package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class leley extends LinearOpMode {

    // ================= HARDWARE =================
    DcMotor fr, fl, bl, br, lift;
    DcMotorEx rwheel, lwheel;
    DcMotor intake;
    Servo rstopper, lstopper;

    IMU imu;

    // ================= DRIVE CONSTANTS =================
    final double DRIVE_POWER_MAX = 0.9;

    // ================= LIFT CONSTANTS =================
    final double TICKS_PER_MM = 48.0625;
    final double MAX_MM = 285.04551365;
    final int MAX_TICKS = (int)(TICKS_PER_MM * MAX_MM);
    final int MIN_TICKS = 0;
    final double UP_POWER = 1.0;
    final double DOWN_POWER = -0.75;

    // ================= FLYWHEEL CONSTANTS =================
    final double TICKS_PER_REV = 537.7;
    final double TARGET_RPM = 215;// for big tri 215
    final double MIN_RPM = 210;// for big tri 210
    final double MAX_RPM = 225;// for big tri 225
    final double SMALL_TARGET_RPM = 250;
    final double SMALL_MIN_RPM = 245;
    final double SMALL_MAX_RPM = 260;
    double targetVelocity;
    double smallTargetVelocity;


    // ================= INTAKE =================
    private static final int INTAKE_STEP = -90;
    private static final double FEED_POWER = -0.80;
    private static final double MOVE_POWER = -0.3;
    private static final double REVERSE_FEED_POWER = 0.45;

    private int intakeTarget = 0;
    private boolean lastA = false;

    private enum IntakeState { IDLE, MANUAL_FEED, MOVING_TO_TARGET, REVERSE_FEED }
    private IntakeState intakeState = IntakeState.IDLE;

    // ================= TOGGLE FLAGS =================
    boolean shooterOn = false;
    boolean aWasPressed = false;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        lift = hardwareMap.get(DcMotor.class, "lift");

        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rstopper = hardwareMap.get(Servo.class, "rstopper");
        lstopper = hardwareMap.get(Servo.class, "lstopper");

        imu = hardwareMap.get(IMU.class, "imu");

        // ---------------- MOTOR DIRECTIONS ----------------
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);

        // ---------------- ENCODER INIT ----------------
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;
        smallTargetVelocity = (SMALL_TARGET_RPM / 60.0) * TICKS_PER_REV;

        // ---------------- IMU INIT ----------------
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        telemetry.addLine("Merged TeleOp Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = lx * Math.cos(heading) - ly * Math.sin(heading);
            double rotatedY = lx * Math.sin(heading) + ly * Math.cos(heading);

            double max = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1);
            double drivePower = DRIVE_POWER_MAX - (0.6 * gamepad1.right_trigger);

            br.setPower(((rotatedY - rotatedX - rx) / max) * drivePower);
            bl.setPower(((rotatedY + rotatedX + rx) / max) * drivePower);
            fr.setPower(((rotatedY + rotatedX - rx) / max) * drivePower);
            fl.setPower(((rotatedY - rotatedX + rx) / max) * drivePower);

            if (gamepad1.y) imu.resetYaw();

            // ================= FLYWHEEL =================
            if (gamepad2.dpad_down) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);

            } else if (gamepad2.dpad_up) {
                rwheel.setVelocity(smallTargetVelocity);
                lwheel.setVelocity(smallTargetVelocity);

            } else if (gamepad2.left_bumper) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }




            // ================= STOPPER =================
            if (gamepad2.b) {
                rstopper.setPosition(0.25);
                lstopper.setPosition(0.25);
            } else {
                rstopper.setPosition(0);
                lstopper.setPosition(0);
            }

            // ================= INTAKE STATE MACHINE =================
            boolean a = gamepad2.right_bumper;

            if (gamepad2.x) {
                intakeState = IntakeState.MANUAL_FEED;
            } else if (gamepad2.y){
                intakeState = IntakeState.REVERSE_FEED;
            } else if (a && !lastA) {
                intakeTarget = intake.getCurrentPosition();
                intakeTarget += INTAKE_STEP;
                intakeState = IntakeState.MOVING_TO_TARGET;
            } else if (!gamepad2.x && !a && intakeState != IntakeState.MOVING_TO_TARGET) {
                intakeState = IntakeState.IDLE;
            }

            lastA = a;

            switch (intakeState) {
                case MANUAL_FEED:
                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intake.setPower(FEED_POWER);
                    break;
                case REVERSE_FEED:
                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intake.setPower(REVERSE_FEED_POWER);
                    break;
                case MOVING_TO_TARGET:
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setTargetPosition(intakeTarget);
                    intake.setPower(MOVE_POWER);
                    if (!intake.isBusy()) intakeState = IntakeState.IDLE;
                    break;
                case IDLE:
                    intake.setPower(0);
                    break;
            }

            // ================= LIFT =================
            int pos = lift.getCurrentPosition();
            if (gamepad1.dpad_up && pos < MAX_TICKS) {
                lift.setPower(UP_POWER);
            } else if (gamepad1.dpad_down && pos > MIN_TICKS) {
                lift.setPower(DOWN_POWER);
            } else {
                lift.setPower(0);
            }

            // ================= TELEMETRY =================
            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM = (lwheel.getVelocity() * 60) / TICKS_PER_REV;
            double avgRPM = (rightRPM + leftRPM) / 2.0;
            boolean rpmReady = (avgRPM >= MIN_RPM && avgRPM <= MAX_RPM);
            boolean smallRpmReady = (avgRPM >= SMALL_MIN_RPM && avgRPM <= SMALL_MAX_RPM);

            //telemetry.addData("Flywheel Target RPM", TARGET_RPM);
            telemetry.addData("Avg RPM", avgRPM);
            //telemetry.addData("RPM Ready", rpmReady);
            //telemetry.addData("SMALL RPM Ready", smallRpmReady);
            //telemetry.addData("Stopper Pos", rstopper.getPosition());
            telemetry.addData("Intake State", intakeState);
            //telemetry.addData("Intake Current", intake.getCurrentPosition());
            //telemetry.addData("Intake Target", intakeTarget);
            telemetry.addData("Lift Pos", pos);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}
