package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class teleyyy extends OpMode{
    DcMotor lwheel, rwheel, intake;
    //fieldpt2 drive = new fieldpt2();
    //double forward, strafe, rotate;

    @Override
    public void init() {
        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        if (gamepad2.x) {
            rwheel.setPower(.5);
            lwheel.setPower(.5);

        } else {
            rwheel.setPower(0);
            lwheel.setPower(0);
        }
        //outakes ball - sets of wheel
        if (gamepad2.b) {
            rwheel.setPower(-.5);
            lwheel.setPower(-.5);
        }



        // vacuum in
        if (gamepad2.b){
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // vacuum out

        if (gamepad2.a){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
        }
    }
