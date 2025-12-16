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
public class fixedTeleop extends LinearOpMode {

    // Chassis
    DcMotor fr, fl, bl, br;

    // Mechanisms
    DcMotor lwheel, rwheel, intake;
    Servo rstopper;

    // IMU (ONLY declared ONCE now!)
    IMU imu;

    // Intake toggle system
    boolean intakeOn = false;
    boolean intakeForward = true;
    boolean xWasPressed = false;
    boolean yWasPressed = false;

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

        rstopper = hardwareMap.servo.get("rstopper");

        // ----------- DIRECTIONS -----------
        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        rstopper.setDirection(Servo.Direction.FORWARD);

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
            double ly = -gamepad1.left_stick_y;  // forward is negative on gamepad
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric transform
            double rotatedX = lx * Math.cos(heading) - ly * Math.sin(heading);
            double rotatedY = lx * Math.sin(heading) + ly * Math.cos(heading);

            double max = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1);
            double drivePower = 0.9 - (0.6 * gamepad1.right_trigger);

            br.setPower(((rotatedY - rotatedX - rx) / max) * drivePower);
            bl.setPower(((rotatedY + rotatedX + rx) / max) * drivePower);
            fr.setPower(((rotatedY + rotatedX - rx) / max) * drivePower);
            fl.setPower(((rotatedY - rotatedX + rx) / max) * drivePower);

            // Reset heading
            if (gamepad1.y) {
                imu.resetYaw();
            }

            // ---------- SHOOTER ----------
            if (gamepad2.a) {
                rwheel.setPower(0.78);
                lwheel.setPower(0.78);
            } else {
                rwheel.setPower(0);
                lwheel.setPower(0);
            }

            // ----------- INTAKE TOGGLE SYSTEM -----------
            // X toggles intake ON/OFF
            if (gamepad2.x && !xWasPressed) {
                intakeOn = !intakeOn;
                sleep(120);  // anti-double-press delay
            }
            xWasPressed = gamepad2.x;

            // Y toggles direction
            if (gamepad2.y && !yWasPressed) {
                intakeForward = !intakeForward;
                sleep(120);
            }
            yWasPressed = gamepad2.y;

            // Apply intake power
            if (intakeOn) {
                intake.setPower(intakeForward ? 0.5 : -0.75);
            } else {
                intake.setPower(0);
            }

            // ---------- STOPPER SERVO ----------
            if (gamepad2.b) {
                rstopper.setPosition(0.57);
            } else {
                rstopper.setPosition(0);
            }

            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}
