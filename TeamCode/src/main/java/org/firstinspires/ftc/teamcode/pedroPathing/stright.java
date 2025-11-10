package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "stright")
public class stright extends LinearOpMode {

    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor fl;

    @Override
    public void runOpMode() throws InterruptedException {


        // this code works when starting from the small triangle,  DONT USE FOR BIG TRIANGLE


        {
            //idk if it will work
            fr=hardwareMap.dcMotor.get("fr");
            bl=hardwareMap.dcMotor.get("bl");
            br=hardwareMap.dcMotor.get("br");
            fl=hardwareMap.dcMotor.get("fl");
            waitForStart();
            fl.setPower(-0.35);
            bl.setPower(0.35);
            fr.setPower(0.35);
            br.setPower(0.35);
            sleep(1500);
        }
    }
}