package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.logging.FileHandler;

@Autonomous(name="ConceptAuto")
public class ConceptAuto extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double max = 0;

    @Override public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("TELEOP - Ready team!", "Press Play Button");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (drive == 0 && strafe == 0 && turn == 0) {
                // Stop the robot
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            } else {
                // Calculate power for each wheel by combining the calculated values for each direction
                leftFrontPower = drive - strafe + turn;
                rightFrontPower = drive + strafe - turn;
                leftBackPower = drive + strafe + turn;
                rightBackPower = drive - strafe - turn;

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
        }
        if (opModeIsActive()) {
            blockOnePath();
        }
    }

    // drive is forward and backward
    // strafe is left and right
    // turn is turning clockwise and anticlockwise
    private void forward(double fMagnitude) {
        drive = fMagnitude;
        strafe = 0;
        turn = 0;
    }
    private void backward(double bMagnitude) {
        drive = -bMagnitude;
        strafe = 0;
        turn = 0;
    }
    private void leftward(double lMagnitude) {
        drive = 0;
        strafe = lMagnitude;
        turn = 0;
    }
    private void rightward(double rMagnitude) {
        drive = 0;
        strafe = -rMagnitude;
        turn = 0;
    }
    private void zeroPower() {
        drive = 0;
        strafe = 0;
        turn = 0;
    }
    private void blockOnePath() {
        forward(10);
        sleep(1000);
        backward(10);
        sleep(1000);
        zeroPower();
    }
}