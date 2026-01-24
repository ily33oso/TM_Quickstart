package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
@TeleOp(name = "DeadWheel_Test_PinPoint_Ready", group = "Test")
public class Dwtest extends OpMode {
    // PinPoint sensor
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        // Map the PinPoint sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure offsets, encoder type, directions, and IMU
        configurePinpoint();

        // Set starting position (0,0,0)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        // Reset position if A is pressed
        if (gamepad1.a) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        // Update the PinPoint readings
        pinpoint.update();

        // Get current robot position
        Pose2D pose2D = pinpoint.getPosition();

        // Telemetry
        telemetry.addLine("=== DEAD WHEEL TEST ===");
        telemetry.addLine("Push your robot around to see it track.");
        telemetry.addLine("Press A to reset the position.");
        telemetry.addData("X coordinate (inches)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (inches)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (degrees)", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public void configurePinpoint() {
        /*
         * Set offsets for your odometry pods relative to robot center
         * X pod: 5.25 in left → 133 mm
         * Y pod: 7 in back → -178 mm
         */
        pinpoint.setOffsets(133.0, -178.0, DistanceUnit.MM);

        // Set the type of odometry pod
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Reset position and IMU before running
        pinpoint.resetPosAndIMU();
    }
}