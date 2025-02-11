package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreeWheelOdometryTest {
    private DcMotor leftEncoder, rightEncoder, strafeEncoder;

    private static final double ODOMETRY_TICKS_PER_REV = 8192; // Adjust based on your encoder specs
    private static final double WHEEL_DIAMETER_INCHES = 2.0; // Adjust based on your odometry wheels
    private static final double ODOMETRY_COUNTS_PER_INCH = ODOMETRY_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    private double x, y, heading;

    public int getLeftEncoderTicks() {
        return leftEncoder.getCurrentPosition();
    }

    public int getRightEncoderTicks() {
        return rightEncoder.getCurrentPosition();
    }

    public int getStrafeEncoderTicks() {
        return strafeEncoder.getCurrentPosition();
    }

    public ThreeWheelOdometryTest(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        strafeEncoder = hardwareMap.get(DcMotor.class, "strafeEncoder");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePose() {
        double leftTicks = leftEncoder.getCurrentPosition();
        double rightTicks = rightEncoder.getCurrentPosition();
        double strafeTicks = strafeEncoder.getCurrentPosition();

        double leftInches = leftTicks / ODOMETRY_COUNTS_PER_INCH;
        double rightInches = rightTicks / ODOMETRY_COUNTS_PER_INCH;
        double strafeInches = strafeTicks / ODOMETRY_COUNTS_PER_INCH;

        x = strafeInches; // Strafe movement
        y = (leftInches + rightInches) / 2.0; // Forward movement
        heading = (rightInches - leftInches) / 12.0; // Adjust based on your robot's track width
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public int getRawX() {
        return strafeEncoder.getCurrentPosition();
    }

    public int getRawY() {
        return (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2;
    }
}
