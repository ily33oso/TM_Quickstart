package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@TeleOp(group = "Test")
public class id extends LinearOpMode {

    private DcMotorEx rwheel;
    private DcMotorEx lwheel;
    private DcMotor intake;

    private Servo lever;
    private Servo stopper;

    // ================= CONSTANTS =================
    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 220;

    private static final double MIN_RPM = 215;
    private static final double MAX_RPM = 235;

    private static final double LEVER_DOWN = 0.0;
    private static final double LEVER_UP   = 0.55;

    private static final double STOPPER_CLOSED = 0.0;
    private static final double STOPPER_OPEN   = 0.5;

    // Timing (tune these)
    private static final long STOPPER_OPEN_TIME = 150; // ms
    private static final long FEED_TIME_MS   = 200;//0g 350
    private static final long SETTLE_TIME_MS = 1200;//og 100
    private static final long SHOT_END_MS    = 1200;


    private double targetVelocity;

    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shooting = false;

    @Override
    public void runOpMode() {

        intake  = hardwareMap.get(DcMotor.class, "intake");
        rwheel  = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel  = hardwareMap.get(DcMotorEx.class, "lwheel");
        lever   = hardwareMap.get(Servo.class, "lever");
        stopper = hardwareMap.get(Servo.class, "stopper");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        stopper.setDirection(Servo.Direction.FORWARD);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        rwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        // Safe defaults
        lever.setPosition(LEVER_DOWN);
        stopper.setPosition(STOPPER_CLOSED);
        intake.setPower(0);

        telemetry.addLine("Flywheel + Stopper + Lever READY");
        telemetry.addLine("A = Spin Up | B = Stop | X = Intake | Y = Shoot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= FLYWHEEL =================
            if (gamepad1.a) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);
            }

            if (gamepad1.b) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }

            // ================= RPM CALC =================
            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM  = (lwheel.getVelocity() * 60) / TICKS_PER_REV;
            double avgRPM   = (rightRPM + leftRPM) / 2.0;

            boolean rpmReady = (avgRPM >= MIN_RPM && avgRPM <= MAX_RPM);

            // ================= START SHOOT =================
            if (gamepad1.y && rpmReady && !shooting) {
                shooting = true;
                shotTimer.reset();
            }

            // ================= SHOOT SEQUENCE =================
            if (shooting) {

                double t = shotTimer.milliseconds();

                if (t < STOPPER_OPEN_TIME) {
                    // Stage 1: open gate
                    stopper.setPosition(STOPPER_OPEN);
                    intake.setPower(0);
                    lever.setPosition(LEVER_DOWN);
                }
                else if (t < STOPPER_OPEN_TIME + FEED_TIME_MS) {
                    // Stage 2: feed
                    stopper.setPosition(STOPPER_OPEN);
                    intake.setPower(-1.0);
                    lever.setPosition(LEVER_DOWN);
                }
                else if (t < STOPPER_OPEN_TIME + FEED_TIME_MS + SETTLE_TIME_MS) {
                    // Stage 3: settle
                    stopper.setPosition(STOPPER_OPEN);
                    intake.setPower(0);
                    lever.setPosition(LEVER_DOWN);
                }
                else if (t < SHOT_END_MS) {
                    // Stage 4: fire lever
                    lever.setPosition(LEVER_UP);
                    intake.setPower(0);
                    stopper.setPosition(STOPPER_OPEN); // keep open during lever
                }
                else {
                    // End shot
                    lever.setPosition(LEVER_DOWN);
                    stopper.setPosition(STOPPER_CLOSED);
                    intake.setPower(0);
                    shooting = false;
                }
            }


            // ================= MANUAL CONTROL =================
            if (!shooting) {
                if (gamepad1.x) {
                    intake.setPower(-0.75);
                } else {
                    intake.setPower(0);
                }

                lever.setPosition(LEVER_DOWN);
                stopper.setPosition(STOPPER_CLOSED);
            }

            // ================= TELEMETRY =================
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("RPM Ready", rpmReady);
            telemetry.addData("Shooting", shooting);
            telemetry.addData("Shot Time (ms)", shotTimer.milliseconds());
            telemetry.update();
        }
    }
}

