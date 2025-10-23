package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class brandonsTeleOp extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    DcMotor lwheel, rwheel, intake;
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.y) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            br.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            bl.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            fr.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            fl.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            //what the sigma - Joel

            // vacuum in and get balls
            if (gamepad2.x) {
                rwheel.setPower(1);
                lwheel.setPower(-1);
                intake.setPower(-1);

            } else {
                rwheel.setPower(0);
                lwheel.setPower(0);
                intake.setPower(0);
            }
            //outakes ball
            if (gamepad2.b) {
                rwheel.setPower(-1);
                lwheel.setPower(1);
                intake.setPower(1);
            } else {
                rwheel.setPower(0);
                lwheel.setPower(0);
                intake.setPower(0);
            }
        }
    }
}


