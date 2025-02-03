package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ThreeWheelOdometryTest {
    // Odometry (Dead Wheel) Encoders
    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;

    // Constants
    private static final double TICKS_PER_REV = 8192; // Rev Through-bore Encoder Example
    private static final double WHEEL_RADIUS = 1.89; // 48mm in inches
    private static final double GEAR_RATIO = 1.0; // No gear reduction
    private static final double LATERAL_DISTANCE = 17.0; // Distance between left and right wheels
    private static final double FORWARD_OFFSET = -3.74; // Distance of front wheel from center

    // Pose Variables
    private double x = 0, y = 0, heading = 0;
    private int lastLeftPos = 0, lastRightPos = 0, lastFrontPos = 0;

    public ThreeWheelOdometryTest(HardwareMap hardwareMap) {
        // Initialize Odometry Encoders (Only Dead Wheels)
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        // Reset and set to run without built-in motor control
        leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Converts encoder ticks to inches
    private double encoderTicksToInches(int ticks) {
        return (WHEEL_RADIUS * 2 * Math.PI * ticks) / TICKS_PER_REV;
    }

    public void updatePose() {
        // Get current encoder positions
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        // Calculate change in encoder values
        double deltaLeft = encoderTicksToInches(leftPos - lastLeftPos);
        double deltaRight = encoderTicksToInches(rightPos - lastRightPos);
        double deltaFront = encoderTicksToInches(frontPos - lastFrontPos);

        // Update last known positions
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastFrontPos = frontPos;

        // Compute change in heading
        double deltaHeading = (deltaRight - deltaLeft) / LATERAL_DISTANCE;

        // Compute forward and strafe movement
        double deltaX = (deltaLeft + deltaRight) / 2.0;
        double deltaY = deltaFront - (deltaHeading * FORWARD_OFFSET);

        // Update pose (using basic kinematics)
        x += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        y += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        heading += deltaHeading;
    }

    // Get current pose
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    // Debugging: Print to telemetry
    public void displayPose(Telemetry telemetry) {
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading (rad)", heading);
        telemetry.update();
    }
}