package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Telop")
public class Teleop extends LinearOpMode {
    // Hardware map variables
    private DcMotor leftFront, leftBack, rightBack, rightFront, linearLeft, linearRight;
    private Servo clawServo, clawArmServo;

    // Motor control variables
    private double drive, strafe, turn;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double max;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        linearLeft = hardwareMap.get(DcMotorEx.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotorEx.class, "linearRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        // Set motor behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;

            // Calculate motor power
            leftFrontPower = drive - strafe + turn;
            rightFrontPower = drive + strafe - turn;
            leftBackPower = drive + strafe + turn;
            rightBackPower = drive - strafe - turn;

            // Normalize motor power
            max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

            leftFront.setPower(leftFrontPower / max);
            rightFront.setPower(rightFrontPower / max);
            leftBack.setPower(leftBackPower / max);
            rightBack.setPower(rightBackPower / max);

            // Linear slide controls
            double linearPower = gamepad1.dpad_up ? -0.8 : gamepad1.dpad_down ? 0.8 : 0;
            linearLeft.setPower(linearPower);
            linearRight.setPower(-linearPower);

            // Claw controls
            if (gamepad1.x) clawServo.setPosition(0.8); // Adjusted for FTC valid range
            if (gamepad1.y) clawServo.setPosition(0.2);

            // Claw arm controls
            if (gamepad1.a) clawArmServo.setPosition(0.6);
            if (gamepad1.b) clawArmServo.setPosition(0.3);

            // Telemetry
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Right Back Power", rightBack.getPower());
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Claw Arm Position", clawArmServo.getPosition());
            telemetry.addData("Linear Left Power", linearLeft.getPower());
            telemetry.addData("Linear Right Power", linearRight.getPower());
            telemetry.update();

            idle();
        }
    }
}