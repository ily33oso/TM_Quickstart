package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "forward")
public class forward extends LinearOpMode {

    DcMotor fl,fr,bl,br;

    @Override
    public void runOpMode() throws InterruptedException {
        //this shoulg work for the big triangle, its enough to leave the tape

        fr=hardwareMap.dcMotor.get("fr");
        bl=hardwareMap.dcMotor.get("bl");
        br=hardwareMap.dcMotor.get("br");
        fl=hardwareMap.dcMotor.get("fl");

        waitForStart();

        fl.setPower(-0.35);
        bl.setPower(0.35);
        fr.setPower(0.35);
        br.setPower(0.35);
        sleep(1250);

    }
}
