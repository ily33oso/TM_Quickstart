package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class thirdtimesacharm extends LinearOpMode {

    private DcMotorEx rwheel;
    private DcMotorEx lwheel;

    DcMotor intake;
    Servo rstopper, lstopper;
    // ================= CONSTANTS =================
    private static final double TICKS_PER_REV = 537.7;
    private static final double TARGET_RPM = 219;

    private static final double MIN_RPM = 214;
    private static final double MAX_RPM = 234;

    private double targetVelocity;

    @Override
    public void runOpMode() {

        // Initialize motors
        rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        intake = hardwareMap.dcMotor.get("intake");

        rstopper = hardwareMap.servo.get("rstopper");
        lstopper = hardwareMap.servo.get("lstopper");

        rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);

        rwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lwheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients
        rwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);
        lwheel.setVelocityPIDFCoefficients(20, 0, 5, 13);

        rwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lwheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Calculate target velocity in ticks/sec
        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        telemetry.addLine("Flywheel PID READY");
        telemetry.addLine("Press A to spin, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= FLYWHEEL CONTROL =================
            if (gamepad2.a) {
                rwheel.setVelocity(targetVelocity);
                lwheel.setVelocity(targetVelocity);
            } else if (gamepad2.left_bumper) {
                rwheel.setVelocity(0);
                lwheel.setVelocity(0);
            }

            if (gamepad2.x){
                intake.setPower(-.75);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.b) {
                rstopper.setPosition(.27);
                lstopper.setPosition(.27);

            } else {
                rstopper.setPosition(0);
                lstopper.setPosition(0);
            }

            // ================= RPM CALC =================
            double rightRPM = (rwheel.getVelocity() * 60) / TICKS_PER_REV;
            double leftRPM  = (lwheel.getVelocity() * 60) / TICKS_PER_REV;
            double avgRPM   = (rightRPM + leftRPM) / 2.0;

            boolean rpmReady = (avgRPM >= MIN_RPM && avgRPM <= MAX_RPM);

            // ================= TELEMETRY =================
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Avg RPM", avgRPM);
            telemetry.addData("RPM Ready", rpmReady);
            telemetry.addData("stopper position", rstopper.getPosition());
            telemetry.update();
        }
    }
}

