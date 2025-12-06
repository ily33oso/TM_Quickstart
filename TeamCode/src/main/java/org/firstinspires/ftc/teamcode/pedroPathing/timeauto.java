package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="blue_timedauto")
public class timeauto extends LinearOpMode {

    DcMotor fr, bl, br, fl;
    DcMotor intake;
    DcMotorEx rwheel, lwheel;

    VoltageSensor battery;

    // hysteresis state
//    private boolean intakeEnabled = false;


    @Override
    public void runOpMode() throws InterruptedException {


        // this code works when starting from the small triangle,  DONT USE FOR BIG TRIANGLE


        {

            fr = hardwareMap.dcMotor.get("fr");
            bl = hardwareMap.dcMotor.get("bl");
            br = hardwareMap.dcMotor.get("br");
            fl = hardwareMap.dcMotor.get("fl");

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

            battery = hardwareMap.voltageSensor.iterator().next();

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
            telemetry.addData("Time", getRuntime());
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
            fr.setPower(.38);
            br.setPower(.38);
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

            //start shooting mechanism
            rwheel.setPower(.75);
            lwheel.setPower(.75);
            sleep(2000);



            //intake feeds artifacts
            rwheel.setPower(.75);
            lwheel.setPower(.75);
            intake.setPower(1);
            sleep(20000);



/*
// Start shooter motors
            rwheel.setPower(0.75);
            lwheel.setPower(0.75);

// Access battery voltage sensor
            VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

// Voltage range where the shooter has enough power
            double minVoltage = 11.0;   // recommended threshold
            double maxWaitTime = 3000;  // failsafe (3 seconds)

            long startTime = System.currentTimeMillis();

            while (opModeIsActive()) {

                double voltage = battery.getVoltage();

                telemetry.addData("Battery Voltage", voltage);
                telemetry.update();

                // If battery voltage is high enough, feed intake
                if (voltage >= minVoltage) {
                    telemetry.addLine("✓ Shooter ready — feeding intake");
                    telemetry.update();
                    intake.setPower(1);
                    break;
                }

                // Safety timeout
                if (System.currentTimeMillis() - startTime > maxWaitTime) {
                    telemetry.addLine("⚠ Timeout reached — feeding intake anyway");
                    telemetry.update();
                    intake.setPower(1);
                    break;
                }
            }

 */

          /*
          /
            // ============================
            // SHOOTER SPINUP
            // ============================
            // start flywheels and do a short ramp (small startup ramp to reduce current spike)
            rampUpFlywheels(0.75, 10, 40); // targetPower, steps, msPerStep

            // give shooter a little settling time
            sleep(300);

            // ============================
            // INTAKE + SHOOTING HYSTERESIS LOOP
            // ============================
            double highThreshold = 12.2;  // battery must rise above this to ENGAGE intake
            double lowThreshold = 11.7;  // battery must drop below this to DISENGAGE

            long duration = 4000;  // run shooting loop for 4 seconds
            long start = System.currentTimeMillis();

            while (opModeIsActive() && System.currentTimeMillis() - start < duration) {

                double volts = battery.getVoltage();
                updateIntakeWithHysteresis(volts, highThreshold, lowThreshold);

                telemetry.addData("Battery", "%.2f V", volts);
                telemetry.addData("Intake State", intakeEnabled ? "ON" : "OFF");
                telemetry.update();

                sleep(40);
            }

            // stop everything
            intake.setPower(0);
            rwheel.setPower(0);
            lwheel.setPower(0);
        }

        // ============================
        // RAMP FUNCTION: gently ramp flywheel power
        // ============================
        private void rampUpFlywheels ( double targetPower, int steps, long msPerStep) throws
        InterruptedException {
            double step = targetPower / Math.max(1, steps);
            double p = step; // start at one step to avoid 0->high jump
            for (int i = 0; i < steps && opModeIsActive(); i++) {
                rwheel.setPower(p);
                lwheel.setPower(p);
                p += step;
                if (p > targetPower) p = targetPower;
                sleep(msPerStep);
            }
            // ensure final power
            rwheel.setPower(targetPower);
            lwheel.setPower(targetPower);
        }

        // ============================
        // HYSTERESIS LOGIC
        // ============================
        /*
         * Updates intakeEnabled state using hysteresis and applies intake power accordingly.
         * @param voltage current battery voltage
         * @param high threshold to enable intake (voltage must be > high to enable)
         * @param low threshold to disable intake (voltage must be < low to disable)
         */
     /*
        public void updateIntakeWithHysteresis(double voltage,double high,double low){

            // turn ON when voltage rises ABOVE high threshold
            if (!intakeEnabled && voltage > high) {
                intakeEnabled = true;
            }

            // turn OFF when voltage falls BELOW low threshold
            if (intakeEnabled && voltage < low) {
                intakeEnabled = false;
            }

            // apply motor power
            if (intakeEnabled) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }
        }

      */

        }

    }

}
