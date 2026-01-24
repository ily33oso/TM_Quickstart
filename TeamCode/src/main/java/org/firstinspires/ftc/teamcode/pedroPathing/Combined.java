package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class Combined extends LinearOpMode {

    // ================= HARDWARE =================
    private DcMotorEx rwheel;
    private DcMotorEx lwheel;
    private DcMotor intake;
    private Servo rstopper, lstopper;

    // ================= CONSTANTS =================
    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 215;
    private static final double MIN_RPM = 210;
    private static final double MAX_RPM = 225;

    private static final int INTAKE_STEP = -90;
    private static final double FEED_POWER = -0.6; // from RoboticsTele
    private static final double MOVE_POWER = -0.3; // from RoboticsTele

    private double targetVelocity;
    private int intakeTarget = 0;
    private boolean lastRB = false;

    // Intake states
    private enum IntakeState {
        IDLE,
        MANUAL_FEED,
        MOVING_TO_TARGET
    }
    private IntakeState intakeState = IntakeState.IDLE;

    @Override
    public void runOpMode() {

        // ================= HARDWARE MAP =================
        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rstopper = hardwareMap.get(Servo.class, "rstopper");
        lstopper = hardwareMap.get(Servo.class, "lstopper");

        // ================= MOTOR SETUP =================
        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);

        rwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        rwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        telemetry.addLine("Flywheel PID READY");
        telemetry.addLine("Press A to spin, LB to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean rb = gamepad2.right_bumper;

            // ================= FLYWHEEL CONTROL =================
            if (gamepad2.a) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);
            } else if (gamepad2.left_bumper) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }

            // ================= STOPPER CONTROL =================
            if (gamepad2.b) {
                rstopper.setPosition(0.25);
                lstopper.setPosition(0.25);
            } else {
                rstopper.setPosition(0);
                lstopper.setPosition(0);
            }

            // ================= INTAKE CONTROL =================
            if (gamepad2.x) {
                intakeState = IntakeState.MANUAL_FEED;
            }
            else if (rb && !lastRB) {
                intakeTarget = intake.getCurrentPosition(); // sync target
                intakeTarget += INTAKE_STEP;
                intakeState = IntakeState.MOVING_TO_TARGET;
            }
            else if (!gamepad2.x && !rb) {
                if (intakeState != IntakeState.MOVING_TO_TARGET) {
                    intakeState = IntakeState.IDLE;
                }
            }

            lastRB = rb;

            // ================= INTAKE STATE MACHINE =================
            switch (intakeState) {
                case MANUAL_FEED:
                    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intake.setPower(FEED_POWER);
                    break;

                case MOVING_TO_TARGET:
                    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setTargetPosition(intakeTarget);
                    intake.setPower(MOVE_POWER);

                    if (!intake.isBusy()) {
                        intake.setPower(0);
                        intakeState = IntakeState.IDLE;
                    }
                    break;

                case IDLE:
                    intake.setPower(0);
                    break;
            }

            // ================= RPM CALC =================
            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM = (lwheel.getVelocity() * 60) / TICKS_PER_REV;
            double avgRPM = (rightRPM + leftRPM) / 2.0;
            boolean rpmReady = (avgRPM >= MIN_RPM && avgRPM <= MAX_RPM);

            // ================= TELEMETRY =================
            telemetry.addData("Flywheel Target RPM", TARGET_RPM);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("RPM Ready", rpmReady);
            telemetry.addData("Stopper Pos", rstopper.getPosition());
            telemetry.addData("Intake State", intakeState);
            telemetry.addData("Intake Current", intake.getCurrentPosition());
            telemetry.addData("Intake Target", intakeTarget);
            telemetry.update();
        }
    }
}
