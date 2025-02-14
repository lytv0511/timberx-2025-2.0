package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="StudioConceptTeleop")
public class StudioConceptTeleop extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private Servo clawServo, clawArmServo;
    private ThreeWheelOdometryTest odometry;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        odometry = new ThreeWheelOdometryTest(hardwareMap);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        odometry.resetEncoders();

        while (opModeIsActive()) {
            odometry.updatePose();

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double leftFrontPower = -(drive - strafe + turn);
            double rightFrontPower = drive + strafe - turn;
            double leftBackPower = -(drive + strafe + turn);
            double rightBackPower = drive - strafe - turn;

            double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

            leftFront.setPower(leftFrontPower / max);
            rightFront.setPower(rightFrontPower / max);
            leftBack.setPower(leftBackPower / max);
            rightBack.setPower(rightBackPower / max);

            // Display odometry values on driver station
            telemetry.addData("X", odometry.getX());
            telemetry.addData("Y", odometry.getY());
            telemetry.addData("Heading", odometry.getHeading());

            telemetry.addData("Left Encoder Ticks", odometry.getLeftEncoderTicks());
            telemetry.addData("Right Encoder Ticks", odometry.getRightEncoderTicks());
            telemetry.addData("Strafe Encoder Ticks", odometry.getStrafeEncoderTicks());

            telemetry.update();
            idle();
        }
    }
}