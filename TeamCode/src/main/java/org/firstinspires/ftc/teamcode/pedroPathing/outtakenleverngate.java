package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Test")
public class outtakenleverngate extends LinearOpMode {

    private DcMotorEx rwheel;
    private DcMotorEx lwheel;
    DcMotor intake;

    Servo lever;
    Servo stopper;

    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 220;

    // RPM window for shooting
    private static final double MIN_RPM = 220;
    private static final double MAX_RPM = 230;

    // Servo positions
    private static final double LEVER_DOWN = 0.0;
    private static final double LEVER_UP   = 0.55;

    private static final double STOPPER_CLOSED = 0.0;
    private static final double STOPPER_OPEN   = 0.5;

    private static final long INTAKE_FEED_TIME_MS = 200;

    private double targetVelocity;
    private ElapsedTime shotTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        intake  = hardwareMap.get(DcMotor.class, "intake");
        rwheel  = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel  = hardwareMap.get(DcMotorEx.class, "lwheel");
        lever   = hardwareMap.get(Servo.class, "lever");
        stopper = hardwareMap.get(Servo.class, "stopper");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        stopper.setDirection(Servo.Direction.REVERSE);


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

        telemetry.addLine("Flywheel PID + Stopper + Lever Ready");
        telemetry.addLine("A = Spin Up | B = Stop | Y = Shoot");
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

            // ================= SHOOT SEQUENCE =================
            if (gamepad1.y && rpmReady) {

                stopper.setPosition(STOPPER_OPEN);

                if (shotTimer.milliseconds() == 0) {
                    shotTimer.reset();
                }

                // Step 1: feed ball
                if (shotTimer.milliseconds() < INTAKE_FEED_TIME_MS) {
                    intake.setPower(-0.75);
                    lever.setPosition(LEVER_DOWN);
                }
                // Step 2: push ball
                else {
                    intake.setPower(0);
                    lever.setPosition(LEVER_UP);
                }

            } else {
                // Reset everything safely
                stopper.setPosition(STOPPER_CLOSED);
                lever.setPosition(LEVER_DOWN);
                intake.setPower(0);
                shotTimer.reset();
            }

            // ================= TELEMETRY =================
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("RPM Ready", rpmReady);
            telemetry.addData("Shot Timer (ms)", shotTimer.milliseconds());
            telemetry.update();
        }
    }
}
