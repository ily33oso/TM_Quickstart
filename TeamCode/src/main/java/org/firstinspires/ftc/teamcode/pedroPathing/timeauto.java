package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="blue_timedauto")
public class timeauto extends LinearOpMode{

    DcMotor fr, bl, br, fl;
    DcMotor intake;
    DcMotorEx rwheel, lwheel;


    @Override
    public void runOpMode() throws InterruptedException {


        // this code works when starting from the small triangle,  DONT USE FOR BIG TRIANGLE


        {

            fr=hardwareMap.dcMotor.get("fr");
            bl=hardwareMap.dcMotor.get("bl");
            br=hardwareMap.dcMotor.get("br");
            fl=hardwareMap.dcMotor.get("fl");

            rwheel = hardwareMap.get(DcMotorEx.class, "rwheel");
            lwheel = hardwareMap.get(DcMotorEx.class, "lwheel");
            intake = hardwareMap.get(DcMotor.class, "intake");

            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);

            lwheel.setDirection(DcMotor.Direction.REVERSE);
            rwheel.setDirection(DcMotor.Direction.FORWARD);
            intake.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addLine("Ready — waiting for start");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            //straight
            fl.setPower(-0.75);
            bl.setPower(-0.75);
            fr.setPower(-0.75);
            br.setPower(-0.75);
            sleep(1600);
            telemetry.addLine("✓ Finished driving straight");
            telemetry.addData("Time",getRuntime());
            telemetry.addData("Power Used", 0.75);
            telemetry.update();
            sleep(300);

            //turns left
            fl.setPower(0);
            bl.setPower(0);
           //fr.setPower(0.185);
            // br.setPower(0.185);
            //fr.setPower(0.225);
            //br.setPower(0.225);
            fr.setPower(.245);
            br.setPower(.245);
            sleep(700);
            telemetry.addLine("✓ Finished turning left");
            telemetry.addData("Time", getRuntime());
            telemetry.addData("Turn Power", 0.225);
            telemetry.update();



/*
            //straight
            fl.setPower(-0.75);
            bl.setPower(-0.75);
            fr.setPower(-0.75);
            br.setPower(-0.75);
            sleep(100);

 */


          /*
            //start shooting mechanism
            rwheel.setPower(.825);
            lwheel.setPower(.825);
            sleep(750);

            //intake feeds artifacts
            rwheel.setPower(.825);
            lwheel.setPower(.825);
            intake.setPower(1);
            sleep(3500);

           */


        }
    }
}
