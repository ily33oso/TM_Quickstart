package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class test extends OpMode {

   // DcMotor fr, fl, br, bl;
    Servo lscoop, rscoop;
    DcMotor lwheel, rwheel, intakes;

    @Override
    public void init() {

        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intakes = hardwareMap.dcMotor.get("intakes");


        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intakes.setDirection(DcMotor.Direction.FORWARD);



       /*
        lscoop = hardwareMap.servo.get("lscoop");
        rscoop = hardwareMap.servo.get("rscoop");


        lscoop.setDirection(Servo.Direction.FORWARD); // clockwise
        rscoop.setDirection(Servo.Direction.REVERSE);
        */
    }

    @Override
    public void loop() {

        /*
        if (gamepad2.b) {
          rscoop.setPosition(.70);
          lscoop.setPosition(.75);
      } else {
          rscoop.setPosition(0);
          lscoop.setPosition(0);
      }

    */

      if (gamepad2.a) {
          rwheel.setPower(.5);
          lwheel.setPower(.5);
          telemetry.log().add(String.valueOf(rwheel.getController().getMotorPower(3)));
          //telemetry.log().add(String.valueOf(rwheel.getPowerFloat()));
          //System.out.println(rwheel.getPowerFloat());

      } else {
          rwheel.setPower(0);
          lwheel.setPower(0);
      }

      if (gamepad2.y) {
          intakes.setPower(1);
      } else {
          intakes.setPower(0);
      }


        if (gamepad2.x) {
            intakes.setPower(-.15);
        } else {
            intakes.setPower(0);
        }






    }
}
