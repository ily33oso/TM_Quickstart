package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class teley extends OpMode{


    DcMotor fl,bl,fr,br;

    DcMotor lwheel, rwheel, intake;

    double forward, strafe, rotate;
    private IMU imu;

    @Override
    public void init() {
      /*
        fl = hardwareMap.get(dcMotor.class, "fl");
        bl = hardwareMap.get(dcMotor.class, "bl");
        fr = hardwareMap.get(dcMotor.class, "fr");
        br = hardwareMap.get(dcMotor.class, "br");
*/

        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class,"imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    @Override
    public void loop() {
        /*
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward,strafe,rotate);

        if (gamepad2.a) {
            intake.setPower(.5);
        } else {
            intake.setPower(0);
        }

        if(gamepad2.y) {
            intake.setPower(-.5);
        } else {
            intake.setPower(0);
        }


        if (gamepad2.b) {
            lwheel.setPower(-1);
            rwheel.setPower(-1);
        } else {
            lwheel.setPower(0);
            rwheel.setPower(0);
        }
        if (gamepad2.x) {
            lwheel.setPower(1);
            rwheel.setPower(1);
        } else {
            lwheel.setPower(0);
            rwheel.setPower(0);
        }
        */
    }


    public void drive(double forward, double strafe, double rotate) {
        double flPower = forward + strafe + rotate;
        double blPower = forward - strafe + rotate;
        double frPower = forward - strafe - rotate;
        double brPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 0.3;

        maxPower = Math.max(maxPower, Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));

        fl.setPower(maxSpeed * (flPower/maxPower));
        bl.setPower(maxSpeed * (blPower/maxPower));
        fr.setPower(maxSpeed * (frPower/maxPower));
        br.setPower(maxSpeed * (brPower/maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeDegrees( theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }
}

