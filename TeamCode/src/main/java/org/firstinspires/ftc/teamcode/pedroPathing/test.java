package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class test extends OpMode {

   // DcMotor lwheel, rwheel, intake, fr, fl, br, bl;
    Servo lscoop, rscoop;
    DcMotor lwheel, rwheel, intake;

    @Override
    public void init() {
       /*
        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");


*/

        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");


        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);



        lscoop = hardwareMap.servo.get("lscoop");
        rscoop = hardwareMap.servo.get("rscoop");


        lscoop.setDirection(Servo.Direction.FORWARD); // clockwise
        rscoop.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
      if (gamepad2.b) {
          rscoop.setPosition(1);
          lscoop.setPosition(1.01);
      } else {
          rscoop.setPosition(0);
          lscoop.setPosition(.01);
      }

      if (gamepad2.a) {
          rwheel.setPower(1);
          lwheel.setPower(1);
      } else {
          rwheel.setPower(0);
          lwheel.setPower(0);
      }

      if (gamepad2.y) {
          intake.setPower(1);
      } else {
          intake.setPower(0);
      }







    }
}
