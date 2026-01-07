package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.pedroPathing.OdometryLocalizer;

@TeleOp(name = "DeadWheel_Test_Odometry", group = "Test")
public class Dwtest extends LinearOpMode {

    // Dead wheel encoders
    private DcMotor xEncoder; // side / strafe
    private DcMotor yEncoder; // back / forward

    // Pedro Pathing odometry localizer
    private OdometryLocalizer odometry;

    // Constants (change these to match your wheels)
    private static final double TICKS_PER_REV = 8192; // GoBilda encoder ticks
    private static final double WHEEL_DIAMETER_INCHES = 1.0; // measure your dead wheel diameter
    private static final double LATERAL_DISTANCE = 5.0; // distance between left/right wheels or X/Y if needed

    @Override
    public void runOpMode() {

        // Hardware map
        xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        yEncoder = hardwareMap.get(DcMotor.class, "yEncoder");

        // Reset encoders
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run without power (just reading encoder ticks)
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse encoders if needed
        // xEncoder.setDirection(DcMotor.Direction.REVERSE);
        // yEncoder.setDirection(DcMotor.Direction.REVERSE);

        // Initialize odometry localizer
        odometry = new OdometryLocalizer(xEncoder, yEncoder, TICKS_PER_REV, WHEEL_DIAMETER_INCHES, LATERAL_DISTANCE);

        waitForStart();

        while (opModeIsActive()) {

            // Update odometry
            odometry.update();

            // Get robot pose (X, Y, Heading)
            Pose2d pose = odometry.getPoseEstimate();

            // Telemetry for encoders and odometry
            telemetry.addLine("=== DEAD WHEEL TEST ===");
            telemetry.addData("Raw X Encoder (Strafe)", xEncoder.getCurrentPosition());
            telemetry.addData("Raw Y Encoder (Forward)", yEncoder.getCurrentPosition());
            telemetry.addLine("--- Odometry (inches) ---");
            telemetry.addData("X (inches)", pose.getX());
            telemetry.addData("Y (inches)", pose.getY());
            telemetry.addData("Heading (rad)", pose.getHeading());
            telemetry.addLine("");
            telemetry.addLine("Push robot by hand:");
            telemetry.addLine("Forward  -> Y changes");
            telemetry.addLine("Strafe   -> X changes");
            telemetry.addLine("Rotate   -> minimal change");

            telemetry.update();
        }
    }
}
