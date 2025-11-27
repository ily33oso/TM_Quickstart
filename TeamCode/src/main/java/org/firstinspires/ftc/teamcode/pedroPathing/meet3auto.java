package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name = "Blue_Audience_OptionC_Encoder_IMU", group = "Autonomous")
public class meet3auto extends LinearOpMode{


    // Drive motors
    DcMotor fl, fr, bl, br;
    DcMotorEx rwheel, lwheel;
    DcMotor intake;


    private BNO055IMU imu;

    private final ElapsedTime runtime = new ElapsedTime();

     static final double TPR = 537.6;            // GoBILDA 312 RPM motors
     static final double WHEEL_DIAM = 4.0;       // 4" wheels
    static final double TICKS_PER_INCH = TPR / (Math.PI * WHEEL_DIAM);

    private static final double DRIVE_SPEED = 0.45;
    private static final double SLOW_DRIVE = 0.35;
    private static final double BACKUP_SPEED = 0.30;

        // Shooter
        private static final double SHOOTER_PWR = 0.725;
        private static final long SHOOTER_SPIN_MS = 1200;
        private static final long FEED_MS = 1200;

        @Override
        public void runOpMode() throws InterruptedException {

            // Map motors
            fl = hardwareMap.get(DcMotor.class, "fl");
            fr = hardwareMap.get(DcMotor.class, "fr");
            bl = hardwareMap.get(DcMotor.class, "bl");
            br = hardwareMap.get(DcMotor.class, "br");

            rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
            lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
            intake = hardwareMap.get(DcMotor.class, "intake");

            // Set directions (your configuration)
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);

            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.FORWARD);

            intake.setDirection(DcMotorSimple.Direction.REVERSE);

            rwheel.setDirection(DcMotorSimple.Direction.FORWARD);
            lwheel.setDirection(DcMotorSimple.Direction.REVERSE);

            // Encoder reset
            for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Shooter RUN_WITHOUT_ENCODER
            rwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // ----------------------------
            // IMU initialization
            // Logo UP, USB LEFT, Power Button UP
            // ----------------------------
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            telemetry.addLine("IMU remap: LOGO UP, USB LEFT, BUTTON UP");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            // ----------------------------
            // AUTONOMOUS PATH
            // ----------------------------

            // 1) Drive forward 48 inches
            encoderDrive(DRIVE_SPEED, 50);

            wait(1500);// new stuff i added

            // 2) Turn RIGHT -40°
            turnToAngle(-40);

            // 3) Drive forward ~12" to firing position
            encoderDrive(SLOW_DRIVE, 12);

            // 4) Spin shooter
            rwheel.setPower(SHOOTER_PWR);
            lwheel.setPower(SHOOTER_PWR);
            sleep(SHOOTER_SPIN_MS);

            // 5) Feed artifact
            intake.setPower(1.0);
            sleep(FEED_MS);
            intake.setPower(0);

            // 6) Stop shooter
            rwheel.setPower(0);
            lwheel.setPower(0);

            // 7) Back up slightly
            encoderDrive(BACKUP_SPEED, -4);

            telemetry.addLine("Auto Complete");
            telemetry.update();
        }

        // ----------------------------
        // Encoder Drive
        // ----------------------------
        private void encoderDrive(double power, double inches) {
            int ticks = (int) Math.round(inches * TICKS_PER_INCH);

            fl.setTargetPosition(fl.getCurrentPosition() + ticks);
            fr.setTargetPosition(fr.getCurrentPosition() + ticks);
            bl.setTargetPosition(bl.getCurrentPosition() + ticks);
            br.setTargetPosition(br.getCurrentPosition() + ticks);

            for (DcMotor m : new DcMotor[]{fl, fr, bl, br})
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            fl.setPower(Math.abs(power));
            fr.setPower(Math.abs(power));
            bl.setPower(Math.abs(power));
            br.setPower(Math.abs(power));

            while (opModeIsActive() &&
                    (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
                idle();
            }

            stopDrive();
        }

        private void stopDrive() {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            for (DcMotor m : new DcMotor[]{fl, fr, bl, br})
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // ----------------------------
        // IMU Turn (P-only)
        // ----------------------------
        public void turnToAngle(double targetAngle) {
            double Kp = 0.02;
            double tolerance = 1.5;
            double maxPower = 0.45;
            double minPower = 0.08;

            while (opModeIsActive()) {

                double heading = imu.getAngularOrientation().firstAngle;
                double error = targetAngle - heading;

                while (error <= -180) error += 360;
                while (error > 180) error -= 360;

                if (Math.abs(error) < tolerance)
                    break;

                double power = Kp * error;

                if (Math.abs(power) > maxPower) power = Math.signum(power) * maxPower;
                if (Math.abs(power) < minPower) power = Math.signum(power) * minPower;

                // Left turn = power > 0
                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            }

            stopDrive();
        }
    }
