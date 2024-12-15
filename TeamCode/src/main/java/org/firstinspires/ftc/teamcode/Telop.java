package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Telop")
public class Telop extends LinearOpMode
{
    // Hardware map variables
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;
    private DcMotor armMotor = null;
    private DcMotor linearLeft = null;
    private DcMotor linearRight = null;
    private CRServo intakeServo = null;
    private Servo trayTiltServoRight = null;
    private Servo trayTiltServoLeft = null;

    // Motor control variables
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double max = 0;
    private ElapsedTime timer = new ElapsedTime();


    @Override public void runOpMode()
    {
        // Initialize the hardware variables and set the direction and zero power behavior
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        linearLeft = hardwareMap.get(DcMotorEx.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotorEx.class, "linearRight");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        trayTiltServoRight = hardwareMap.get(Servo.class, "trayTiltServoRight");
        trayTiltServoLeft = hardwareMap.get(Servo.class, "trayTiltServoLeft");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // All the configuration for the OTOS is done in this helper method, check it out!

        //*************************************************************************
        // Wait for driver to press start
        telemetry.addData("TELEOP - Ready team!", "Press Play Button");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {

            //*************************************************************************
            //  CONTROLS
            //*************************************************************************

            // GamePad 1 - Drive and April Tags
            if (Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x) > 0) {

                // Drive manually using the left stick for forward/backward/strafing, and the right stick for turning.
                drive = gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                turn = -gamepad1.right_stick_x; // Reduce turn rate
            }

            if (gamepad1.y) {
                while (gamepad1.y) {
                    intakeServo.setPower(100);
                }
                intakeServo.setPower(0);
            } else if (gamepad1.x) {
                while (gamepad1. x) {
                    intakeServo.setPower(-100);
                }
                intakeServo.setPower(0);
            }


            if (gamepad1.a && !gamepad1.b) {
                armMotor.setPower(10); // Move the arm up
            } else {
                armMotor.setPower(0);
            }

            if (gamepad1.b && !gamepad1.a) {
                armMotor.setPower(-10);
            } else {
                armMotor.setPower(0);
            }

            if (gamepad1.dpad_up) {
                double tiltMagnitude = 0.02;
                long tiltDurationMs = 500;
                trayTiltServoLeft.setDirection(Servo.Direction.REVERSE);
                double currentPosition = 0.0;
                trayTiltServoLeft.setPosition(tiltMagnitude);
                timer.reset();
                while (timer.milliseconds() < tiltDurationMs) {
                    // Wait for the tilt duration
                }
                trayTiltServoLeft.setPosition(currentPosition);
            }

            //*************************************************************************
            //  MOVE ROBOT
            //*************************************************************************

            // Move the robot using calculated values or values from controller
            if (drive == 0 && strafe == 0 && turn == 0) {
                // Stop the robot
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            } else {
                // Calculate power for each wheel by combining the calculated values for each direction
                leftFrontPower = drive -strafe +turn;
                rightFrontPower = drive +strafe -turn;
                leftBackPower = drive +strafe +turn;
                rightBackPower = drive -strafe -turn;

                // Find the maximum value of all the wheel powers
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                // If the maximum value is greater than 1 (max power you can send to wheels), then normalize all the wheel powers by dividing by the maximum value.
                // The normalization process keeps the same ratio of power between the wheels, but makes sure that no wheel power is greater than 1.
                if (max > 1) {
                    leftFrontPower = leftFrontPower / max;
                    rightFrontPower = rightFrontPower / max;
                    leftBackPower = leftBackPower / max;
                    rightBackPower = rightBackPower / max;
                }

                // Send power to wheels
                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftBack.setPower(leftBackPower);
                rightBack.setPower(rightBackPower);
            }

            // Reset the calculated values so the robot stops if no new values are calculated
            drive = 0;
            strafe = 0;
            turn = 0;

            // It is best practice to always call idle() at the bottom of your loops.
            // Yielding CPU Time: It gives time back to the system to process other tasks, such as communications and hardware updates.
            // Handling Stop Requests: It checks if a stop has been requested for your OpMode and facilitates a graceful shutdown if needed.
            // FTC SDK Compliance: It's a recommended practice in FTC programming to ensure your OpMode cooperates well with the overall control system.
            idle();
        }
    }

}