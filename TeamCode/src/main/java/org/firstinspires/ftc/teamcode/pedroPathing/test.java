package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (group = "Test")
public class test extends OpMode {

   // DcMotor fr, fl, br, bl;
   // Servo lscoop, rscoop; //270 degrees

    //CRServo lspin, rspin; //360 degrees
    DcMotor lwheel, rwheel, intake, lift;
    //DcMotor lift;

    Servo rstopper, lstopper;
    Servo lever;
    Servo compressor;
    //rscooper;



    @Override
    public void init() {

        lwheel = hardwareMap.dcMotor.get("lwheel");
        rwheel = hardwareMap.dcMotor.get("rwheel");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");


        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);


        rstopper=hardwareMap.servo.get("rstopper");
        lstopper=hardwareMap.servo.get("lstopper");
        lever = hardwareMap.servo.get("lever");
        compressor = hardwareMap.servo.get("compressor");
      //  rscooper=hardwareMap.servo.get("rscooper");//1


        rstopper.setDirection(Servo.Direction.REVERSE);
        lstopper.setDirection(Servo.Direction.FORWARD);
        lever.setDirection(Servo.Direction.FORWARD);
        compressor.setDirection(Servo.Direction.REVERSE);
//rscooper.setDirection(Servo.Direction.REVERSE);


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


        if (gamepad2.left_bumper) {
            lever.setPosition(.47);
        } else {
            lever.setPosition(0);
        }


        if (gamepad2.b) {
            rstopper.setPosition(.24);
            lstopper.setPosition(.24);

        } else {
            rstopper.setPosition(0);
            lstopper.setPosition(0);

        }

        /*
        if (gamepad2.right_bumper){
            compressor.setPosition(.47);
        } else {
            compressor.setPosition(0);
        }


         */


/*
        if (gamepad2.left_bumper) {
            rscooper.setPosition(1);

        } else {
            rscooper.setPosition(0);

        }


*/
      if (gamepad2.a) {
          rwheel.setPower(.725);
          lwheel.setPower(.725);
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

        if (gamepad2.x) {
            intake.setPower(1);
        } else if (gamepad2.y) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }



        }








    }
