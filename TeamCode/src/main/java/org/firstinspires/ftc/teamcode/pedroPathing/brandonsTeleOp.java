package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class brandonsTeleOp extends LinearOpMode {
    DcMotor fr,fl,bl,br; //chasis

    DcMotor lwheel, rwheel, intake ; //mechanism
    //CRServo lspin, rspin;// mechanism
    Servo rstopper;//mechanism


    //Servo lscoop,rscoop;
    BNO055IMU imu;
    //BN0055IMUNew imu;

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        br = hardwareMap.dcMotor.get("br");//1 control, red, motor
        bl = hardwareMap.dcMotor.get("bl");//2 control, orange, motor
        fr = hardwareMap.dcMotor.get("fr");//3 control, black, motor
        fl = hardwareMap.dcMotor.get("fl");//0 control, pink, motor

        lwheel = hardwareMap.dcMotor.get("lwheel");// teal crown, 3 on expansion, motor
        rwheel = hardwareMap.dcMotor.get("rwheel");// pink bow, 2 on expansion, motor
        intake = hardwareMap.dcMotor.get("intake");//0 on expansion, motor
      //  intake2 = hardwareMap.dcMotor.get("intake");//0 on expansion, motor
        rstopper=hardwareMap.servo.get("rstopper");//0 on control, servo


        lwheel.setDirection(DcMotor.Direction.REVERSE);
        rwheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
     //   intake2.setDirection(DcMotor.Direction.FORWARD);
        rstopper.setDirection(Servo.Direction.FORWARD);

     //   lspin = hardwareMap.crservo.get("lspin");
       // rspin = hardwareMap.crservo.get("rspin");

        //lspin.setDirection(CRServo.Direction.FORWARD);
        //rspin.setDirection(CRServo.Direction.REVERSE);


/*
        lscoop = hardwareMap.servo.get("lscoop");
        rscoop = hardwareMap.servo.get("rscoop");


        lscoop.setDirection(Servo.Direction.FORWARD); // clockwise
        rscoop.setDirection(Servo.Direction.REVERSE);
*/

        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);


        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
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
/*
            if (gamepad2.b) {
                rscoop.setPosition(.70);
                lscoop.setPosition(.75);
            } else {
                rscoop.setPosition(0);
                lscoop.setPosition(0);
            }
*/
/*
            if (gamepad2.b) {
                lspin.setPower(1);
                rspin.setPower(1);
            } else {
                lspin.setPower(0);
                rspin.setPower(0);
            }
*/

            if (gamepad2.a) {
                rwheel.setPower(.75);//for regular teleop use .825 for consitent shooing but for auto .75

                telemetry.log().add(String.valueOf(rwheel.getController().getMotorPower(3)));
                //telemetry.log().add(String.valueOf(rwheel.getPowerFloat()));
                //System.out.println(rwheel.getPowerFloat());
                lwheel.setPower(.75);
            } else {
                rwheel.setPower(0);
                lwheel.setPower(0);
            }

            //if (gamepad2.x) {
                //intake.setPower(1);
//intake2.setPower(1);
            //} else {
             //   intake.setPower(0);
               // intake2.setPower(0);
           // }


            //if (gamepad2.y) {
              //  intake.setPower(-1);
              //  intake2.setPower(-1);
           // } else {
             //   intake.setPower(0);


                //intake2.setPower(0);
           // }
            if (gamepad2.x) {
                intake.setPower(1);    // forward
            } else if (gamepad2.y) {
                intake.setPower(-1);    // reverse
            } else {
                intake.setPower(0);     // idle
            }



            if (gamepad2.b) {
                rstopper.setPosition(.575);

            } else {
                rstopper.setPosition(0);

            }


        }
    }
}



