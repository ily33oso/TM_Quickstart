package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class test extends OpMode {

   // DcMotor fr, fl, br, bl;
   // Servo lscoop, rscoop; //270 degrees
    //CRServo lspin, rspin; //360 degrees
    DcMotor lwheel, rwheel, intake;

    @Override
    public void init() {

        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");


        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

/*
        lspin = hardwareMap.crservo.get("lspin");
        rspin = hardwareMap.crservo.get("rspin");

        lspin.setDirection(CRServo.Direction.FORWARD);
        rspin.setDirection(CRServo.Direction.REVERSE);

*/

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
          rwheel.setPower(.7);
          lwheel.setPower(.70);
          telemetry.log().add(String.valueOf(rwheel.getController().getMotorPower(3)));
          //telemetry.log().add(String.valueOf(rwheel.getPowerFloat()));
          //System.out.println(rwheel.getPowerFloat());

      } else {
          rwheel.setPower(0);
          lwheel.setPower(0);
      }

/*
      if (gamepad2.b) {
          lspin.setPower(1);
          rspin.setPower(1);
      } else {
          lspin.setPower(0);
          rspin.setPower(0);
      }

*/

      if (gamepad2.y) {
          intake.setPower(1);
      } else {
          intake.setPower(0);
      }


        if (gamepad2.x) {
            intake.setPower(-.15);
        } else {
            intake.setPower(0);
        }






    }
}
