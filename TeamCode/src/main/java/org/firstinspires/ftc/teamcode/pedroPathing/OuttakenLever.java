package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class OuttakenLever extends LinearOpMode {

    private DcMotorEx rwheel;
    private DcMotorEx lwheel;
    DcMotor intake;
    Servo lever;

    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 220;

    // RPM window for shooting
    private static final double MIN_RPM = 210;
    private static final double MAX_RPM = 220;

    // Servo positions
    private static final double LEVER_DOWN = 0.0;
    private static final double LEVER_UP = 0.55;

    private double targetVelocity;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        lever = hardwareMap.get(Servo.class, "lever");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        rwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        rwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        lever.setPosition(LEVER_DOWN);

        telemetry.addLine("Flywheel PID + Lever Ready");
        telemetry.addLine("A = Spin Up | B = Stop | Y = Fire Lever");
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

            // ================= INTAKE =================
            if (gamepad1.x) {
                intake.setPower(-0.75);
            } else {
                intake.setPower(0);
            }

            // ================= RPM CALC =================
            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM = (lwheel.getVelocity() * 60) / TICKS_PER_REV;
            double avgRPM = (rightRPM + leftRPM) / 2.0;

            boolean rpmReady = (avgRPM >= MIN_RPM && avgRPM <= MAX_RPM);

            // ================= LEVER LOGIC =================
            if (gamepad1.y && rpmReady) {
                lever.setPosition(LEVER_UP);
            } else {
                lever.setPosition(LEVER_DOWN);
            }

            // ================= TELEMETRY =================
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Right RPM", rightRPM);
            telemetry.addData("Left RPM", leftRPM);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("RPM Ready", rpmReady);
            telemetry.update();
        }
    }
}
