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

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
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

    @Override public void runOpMode() {
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

        if (opModeIsActive()) {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);

            sleep(4000);

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }
}
