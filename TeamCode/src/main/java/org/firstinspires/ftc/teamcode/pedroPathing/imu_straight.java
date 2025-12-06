package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="IMU Straight Drive (No AxesOrder)", group="Auto")
public class imu_straight extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        // ----------------------------
        // Hardware Map
        // ----------------------------
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        // Reverse left side
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        // ----------------------------
        // IMU Setup (NO AXES ORDER NEEDED)
        // ----------------------------
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        telemetry.addLine("IMU Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            telemetry.addLine("Driving Straight...");
            telemetry.update();

            driveStraightIMU(0.4, 48); // drive 48 inches
        }
    }

    // ---------------------------------------------------------
    // Straight Drive using IMU Yaw (NO AxesOrder)
    // ---------------------------------------------------------
    private void driveStraightIMU(double power, double inches) {

        double TICKS_PER_REV = 537.7;  // goBILDA 312 rpm
        double WHEEL_DIAMETER = 4.0;   // inches
        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

        int target = (int)(inches * TICKS_PER_INCH);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(target);
        fr.setTargetPosition(target);
        bl.setTargetPosition(target);
        br.setTargetPosition(target);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {

            double heading = getHeading();
            double correction = heading * 0.03;  // tuning constant

            double leftPower = power - correction;
            double rightPower = power + correction;

            fl.setPower(leftPower);
            bl.setPower(leftPower);
            fr.setPower(rightPower);
            br.setPower(rightPower);

            telemetry.addData("Heading", heading);
            telemetry.addData("Correction", correction);
            telemetry.addData("Target", target);
            telemetry.addData("FL Position", fl.getCurrentPosition());
            telemetry.update();
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // ---------------------------------------------------------
    // Get Heading (NO AxesOrder, NO AxesReference)
    // ---------------------------------------------------------
    private double getHeading() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;  // default yaw
    }
}
