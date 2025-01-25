package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="StudioTeleop")
public class StudioTeleop extends LinearOpMode {
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

        boolean isHoverMode = false;
        boolean isRetractMode = false;
        boolean isModeTwo = false;
        boolean leftStickButtonPressed = false;
        boolean isTurnInProgress = false;
        boolean holdSlidesMode = false;
        boolean isNormalGrabClosed = false;
        boolean isForceGrabClosed = false;
        boolean yButtonPressed = false;
        boolean xButtonPressed = false;

        linearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Toggle between Mode 1 and Mode 2 (Inverting Movement)
            if (gamepad1.left_stick_button && !leftStickButtonPressed) {
                isModeTwo = !isModeTwo;
                leftStickButtonPressed = true;
            }
            if (!gamepad1.left_stick_button) {
                leftStickButtonPressed = false;
            }

            // Toggle Normal Grab (Y button)
            if (gamepad1.y && !yButtonPressed) {
                isNormalGrabClosed = !isNormalGrabClosed;
                isForceGrabClosed = false;
                clawServo.setPosition(isNormalGrabClosed ? 0.4 : 0.6);
                yButtonPressed = true;
            }
            if (!gamepad1.y) {
                yButtonPressed = false;
            }

            // Toggle Force Grab (X button)
            if (gamepad1.x && !xButtonPressed) {
                isForceGrabClosed = !isForceGrabClosed;
                isNormalGrabClosed = false;
                clawServo.setPosition(isForceGrabClosed ? 0.4 : 0.7);
                xButtonPressed = true;
            }
            if (!gamepad1.x) {
                xButtonPressed = false;
            }

            boolean sweepingMode = false;
            boolean sweepingDirection = true;
            long lastSweepTime = System.currentTimeMillis();
            long sweepInterval = 500; // Adjust speed of sweeping motion

            if (gamepad1.b) {
                clawArmServo.setPosition(0.1);
                clawServo.setPosition(0.4); // Open claw
                sweepingMode = true;
                telemetry.update();

                while (gamepad1.b && opModeIsActive()) {
                    double sweepPower = 0.5;

                    // Sweep in one direction
                    leftFront.setPower(sweepPower);
                    leftBack.setPower(sweepPower);
                    rightFront.setPower(sweepPower);
                    rightBack.setPower(sweepPower);
                    sleep(500);

                    // Reverse sweeping direction
                    leftFront.setPower(-sweepPower);
                    leftBack.setPower(-sweepPower);
                    rightFront.setPower(-sweepPower);
                    rightBack.setPower(-sweepPower);
                    sleep(500);
                }

                // Stop movement when B is released
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }


            // 180 Degree Turn (A button)
            if (gamepad1.a && !isTurnInProgress) {
                isTurnInProgress = true;
                isModeTwo = !isModeTwo; // Invert movement mode
                clawArmServo.setPosition(0.9);

                double turnPower = 1.0;
                long turnTime = 825;

                leftFront.setPower(turnPower);
                rightFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightBack.setPower(turnPower);

                sleep(turnTime);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                isTurnInProgress = false;
            }

            // Claw Arm Hover & Retract (Triggers)
            if (gamepad1.right_trigger > 0) {
                isHoverMode = true;
                clawArmServo.setPosition(0.2);
            }
            if (gamepad1.right_trigger == 0 && isHoverMode) {
                isHoverMode = false;
                clawArmServo.setPosition(0.1);
            }

            if (gamepad1.left_trigger > 0) {
                isRetractMode = true;
                clawArmServo.setPosition(0.2);
            }
            if (gamepad1.left_trigger == 0 && isRetractMode) {
                isRetractMode = false;
                clawArmServo.setPosition(0.9);
            }

            // Slides Control with D-Pad
            if (gamepad1.dpad_up) {
                linearLeft.setPower(-1);
                linearRight.setPower(1);
            } else if (gamepad1.dpad_down) {
                linearLeft.setPower(0.5);
                linearRight.setPower(-0.5);
            } else if (gamepad1.dpad_right) {
                holdSlidesMode = false;
            } else if (gamepad1.dpad_left) {
                holdSlidesMode = false;
            } else if (gamepad1.left_bumper) {
                holdSlidesMode = true;
            }

            if (holdSlidesMode && !gamepad1.dpad_up && !gamepad1.dpad_down) {
                linearLeft.setPower(-0.1);
                linearRight.setPower(0.1);
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                linearLeft.setPower(0);
                linearRight.setPower(0);
            }

            // Movement logic
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (isModeTwo) {
                drive = -drive;
                strafe = -strafe;
            }

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

            // Telemetry updates
            telemetry.addData("Mode", isModeTwo ? "Scoring Mode" : "Pickup Mode");;
            telemetry.addData("Hold Slides Mode", holdSlidesMode ? "Active" : "Inactive");
            telemetry.addData("Aiming Mode", isHoverMode ? "Aiming hover" : "Ground");
            telemetry.addData("Retract Mode", isRetractMode ? "Retract hover" : "Retracted");

            telemetry.addData("Motor Positions",
                    "LF: %d | LB: %d | RF: %d | RB: %d",
                    leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightFront.getCurrentPosition(), rightBack.getCurrentPosition());

            telemetry.addData("Slide Motor Positions",
                    "LL: %d | LR: %d",
                    linearLeft.getCurrentPosition(), linearRight.getCurrentPosition());

            telemetry.addData("Motor Powers",
                    "LF: %.2f | LB: %.2f | RF: %.2f | RB: %.2f",
                    leftFront.getPower(), leftBack.getPower(), rightFront.getPower(), rightBack.getPower());

            telemetry.addData("Slide Motor Powers",
                    "LL: %.2f | LR: %.2f",
                    linearLeft.getPower(), linearRight.getPower());

            telemetry.update();

            telemetry.update();
            idle();
        }
    }
}