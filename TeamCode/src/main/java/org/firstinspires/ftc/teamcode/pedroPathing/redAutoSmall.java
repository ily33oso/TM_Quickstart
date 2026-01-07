package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "red auto - small triangle - chasis")
public class redAutoSmall extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         fr,fl,br,bl   = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09448819 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                fl.getCurrentPosition(),
                fr.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12,12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, -24, -24,4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double flInches, double frInches, double blInches, double brInches,
                             double timeoutS) {
        int newflTarget;
        int newfrTarget;
        int newblTarget;
        int newbrTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newflTarget = fl.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
            newfrTarget = fr.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
            newblTarget = bl.getCurrentPosition() + (int)(blInches * COUNTS_PER_INCH);
            newbrTarget = br.getCurrentPosition() + (int)(brInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newflTarget);
            fr.setTargetPosition(newfrTarget);
            bl.setTargetPosition(newblTarget);
            br.setTargetPosition(newbrTarget);

            // Turn On RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newflTarget,  newfrTarget, newblTarget,  newbrTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        fl.getCurrentPosition(), fr.getCurrentPosition(),bl.getCurrentPosition(), fr.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            // Turn off RUN_TO_POSITION
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}


/*
    // ---- DRIVE MOTORS ----
    DcMotor fl, fr, bl, br;

    // ---- ENCODER CONSTANTS ----
    // goBILDA 5203, 312 RPM, 19.2:1
    static final double TICKS_PER_REV = 10362.0;

    // 104mm GripForce mecanum wheels
    static final double WHEEL_DIAMETER_IN = 4.09;

    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    @Override
    public void runOpMode() {

        // ---- HARDWARE MAP ----
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // ---- ALL MOTORS FORWARD (IMPORTANT) ----
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---- BRAKE ----
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        telemetry.addLine("Encoder Forward Test Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // DRIVE FORWARD 12 INCHES
            driveForward(12, 0.6);

            sleep(2000);
        }
    }

    // ================= METHODS =================

    private void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ---- FORWARD DRIVE ----
    private void driveForward(double inches, double power) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        // LEFT side negative, RIGHT side positive
        fl.setTargetPosition(ticks);
        bl.setTargetPosition(ticks);
        fr.setTargetPosition(-ticks);
        br.setTargetPosition(-ticks);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        while (opModeIsActive() &&
                (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();
        resetEncoders();
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

 */

