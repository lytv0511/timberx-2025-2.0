package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreeWheelOdometryTest {
    private DcMotorEx leftEncoderMotor, rightEncoderMotor, strafeEncoderMotor;
    private double x = 0, y = 0, heading = 0;

    private int leftEncoderOffset = 0;
    private int rightEncoderOffset = 0;
    private int strafeEncoderOffset = 0;

    private static final double STRAFE_CONVERSION_FACTOR = 0.001; // Placeholder value
    private static final double FORWARD_CONVERSION_FACTOR = 0.001; // Placeholder value
    private static final double HEADING_CONVERSION_FACTOR = 0.001; // Placeholder value

    public ThreeWheelOdometryTest(HardwareMap hardwareMap) {
        leftEncoderMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoderMotor = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        strafeEncoderMotor = hardwareMap.get(DcMotorEx.class, "strafeEncoder");

        resetEncoders();
    }

    public void resetEncoders() {
        leftEncoderOffset = leftEncoderMotor.getCurrentPosition();
        rightEncoderOffset = rightEncoderMotor.getCurrentPosition();
        strafeEncoderOffset = strafeEncoderMotor.getCurrentPosition();

        x = 0;
        y = 0;
        heading = 0;
    }

    public void updatePose() {
        double leftEncoder = leftEncoderMotor.getCurrentPosition() - leftEncoderOffset;
        double rightEncoder = rightEncoderMotor.getCurrentPosition() - rightEncoderOffset;
        double strafeEncoder = strafeEncoderMotor.getCurrentPosition() - strafeEncoderOffset;

        double deltaX = strafeEncoder * STRAFE_CONVERSION_FACTOR;
        double deltaY = (leftEncoder + rightEncoder) / 2.0 * FORWARD_CONVERSION_FACTOR;
        double deltaHeading = (rightEncoder - leftEncoder) * HEADING_CONVERSION_FACTOR;

        x += deltaX;
        y += deltaY;
        heading += deltaHeading;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public int getLeftEncoderTicks() { return leftEncoderMotor.getCurrentPosition() - leftEncoderOffset; }
    public int getRightEncoderTicks() { return rightEncoderMotor.getCurrentPosition() - rightEncoderOffset; }
    public int getStrafeEncoderTicks() { return strafeEncoderMotor.getCurrentPosition() - strafeEncoderOffset; }
}