package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class teleyyy extends OpMode{
    DcMotor lwheel, rwheel, intake;
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    //fieldpt2 drive = new fieldpt2();
    //double forward, strafe, rotate;

    @Override
    public void init() {
        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
    }

    @Override
    public void loop() {
        // Front and Back Left
        if (Math.abs(gamepad1.left_stick_y) > .2){
            fl.setPower(gamepad1.left_stick_y * 1);
            bl.setPower(gamepad1.left_stick_y * -1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
        }
        // Front anf Back Right
        if (Math.abs(gamepad1.right_stick_y) > .2){
            fr.setPower(gamepad1.left_stick_y * -1);
            br.setPower(gamepad1.left_stick_y * 1);
        } else {
            fr.setPower(0);
            br.setPower(0);
        }
        // Strafing Right
        if (gamepad1.right_bumper) {
            fl.setPower(1);
            bl.setPower(1);
            fr.setPower(1);
            br.setPower(1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
        // Strafing Left
        if (gamepad1.left_bumper) {
            fl.setPower(1);
            bl.setPower(1);
            fr.setPower(1);
            br.setPower(1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
        // vacuum in and get balls
        if (gamepad2.x) {
            rwheel.setPower(1);
            lwheel.setPower(-1);
            intake.setPower(1);

        } else {
            rwheel.setPower(0);
            lwheel.setPower(0);
            intake.setPower(0);
        }
        //outakes ball - sets of wheel
        if (gamepad2.b) {
            rwheel.setPower(1);
            lwheel.setPower(-1);
            intake.setPower(1);
        } else{
            rwheel.setPower(0);
            lwheel.setPower(0);
            intake.setPower(0);
        }

        }
    }
