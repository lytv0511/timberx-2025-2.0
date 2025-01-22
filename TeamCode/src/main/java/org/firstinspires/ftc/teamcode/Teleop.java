package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop")
public class Teleop extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightBack, rightFront, linearLeft, linearRight;
    private Servo clawServo, clawArmServo;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        linearLeft = hardwareMap.get(DcMotorEx.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotorEx.class, "linearRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Flags for mode tracking
        boolean isHoverMode = false;
        boolean isRetractMode = false;

        linearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Right trigger (Hover mode) logic
            if (gamepad1.right_trigger > 0 && !isHoverMode) {
                isHoverMode = true;
                clawArmServo.setPosition(0.6); // Hover position
            }
            if (gamepad1.right_trigger == 0 && isHoverMode) {
                isHoverMode = false;
                clawArmServo.setPosition(0.72); // Down position when released
            }

            // Left trigger (Retract mode) logic
            if (gamepad1.left_trigger > 0 && !isRetractMode) {
                isRetractMode = true;
                clawArmServo.setPosition(0.6); // Hover position
            }
            if (gamepad1.left_trigger == 0 && isRetractMode) {
                isRetractMode = false;
                clawArmServo.setPosition(0.2); // Retract position when released
            }

            // Movement logic (same for both modes)
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

            // Linear slide controls
            double linearPower = 0;
            if (gamepad1.dpad_up) linearPower = -0.8;
            if (gamepad1.dpad_down) linearPower = 0.8;

            linearLeft.setPower(linearPower);
            linearRight.setPower(-linearPower);

            // Claw controls
            if (gamepad1.a) clawServo.setPosition(0.7);
            if (gamepad1.b) clawServo.setPosition(0.4);

            telemetry.addData("Hover Mode", isHoverMode ? "Active" : "Inactive");
            telemetry.addData("Retract Mode", isRetractMode ? "Active" : "Inactive");
            telemetry.addData("Left Front Power", leftFront.getPower());
            telemetry.addData("Left Front Position", leftFront.getCurrentPosition());
            telemetry.addData("Right Front Power", rightFront.getPower());
            telemetry.addData("Right Front Position", rightFront.getCurrentPosition());
            telemetry.addData("Left Back Power", leftBack.getPower());
            telemetry.addData("Left Back Position", leftBack.getCurrentPosition());
            telemetry.addData("Right Back Power", rightBack.getPower());
            telemetry.addData("Right Back Position", rightBack.getCurrentPosition());
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Claw Arm Position", clawArmServo.getPosition());
            telemetry.addData("Linear Left Power", linearLeft.getPower());
            telemetry.addData("Linear Left Position", linearLeft.getCurrentPosition());
            telemetry.addData("Linear Right Power", linearRight.getPower());
            telemetry.addData("Linear Right Position", linearRight.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}