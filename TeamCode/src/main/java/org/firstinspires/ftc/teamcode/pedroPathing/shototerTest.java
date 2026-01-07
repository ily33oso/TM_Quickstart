package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class shototerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
        DcMotorEx lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {

            rwheel.setPower(0.75);
            lwheel.setPower(0.75);

            telemetry.addData("Battery Voltage", battery.getVoltage());
            telemetry.addLine("Shooter spinning...");
            telemetry.update();
        }
    }
}