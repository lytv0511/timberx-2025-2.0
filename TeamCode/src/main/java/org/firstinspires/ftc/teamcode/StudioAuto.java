package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "StudioAuto", group = "Autonomous")
public class StudioAuto extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private Servo clawServo, clawArmServo;
    private ThreeWheelOdometryTest odometry;
    private static final double strafeCorrectionFactor = 10.0 / 9.0;
    private static final double forwardBackwardCorrectionFactor = 10.0 / 11.0;
    private static final double angleCorrectionFactor = 90.0 / 45.0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry = new ThreeWheelOdometryTest(hardwareMap);

        telemetry.addData("Status", "Waiting for start command...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                odometry.updatePose();

                double x = odometry.getX();
                double y = odometry.getY();
                double heading = odometry.getHeading();

                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("Heading (rad)", heading);
                telemetry.update();
            }

            clawArmServo.setPosition(0.87);
            sleep(500);

            strafeLeft(0.5, 36);
            driveBackward(0.5, 54);
            turnLeft(0.3, 45);
            driveBackward(0.5, 13);

            clawArmServo.setPosition(0.87);
            clawServo.setPosition(0.4);
            sleep(1000);
            clawArmServo.setPosition(0.17);
            driveForward(0.5, 13);
            driveForward(0.5, 3);
            turnLeft(0.3, 45);

            clawArmServo.setPosition(0.07);
            sleep(1000);
            driveForward(0.5, 14);
            clawServo.setPosition(0.7);
            sleep(1000);
            clawArmServo.setPosition(0.88);
            sleep(1000);
            driveBackward(0.5, 14);
            driveBackward(0.5, 3);
            turnRight(0.3, 45);

            driveBackward(0.5, 13);
            clawArmServo.setPosition(0.87);
            clawServo.setPosition(0.4);
            sleep(1000);
            clawArmServo.setPosition(0.17);
            driveForward(0.5, 13);
            driveForward(0.5, 3);
            turnLeft(0.3, 45);

            strafeLeft(0.5, 31);
            clawArmServo.setPosition(0.07);
            sleep(1000);
            driveForward(0.5, 14);
            clawServo.setPosition(0.7);
            sleep(1000);
            clawArmServo.setPosition(0.87);
            sleep(1000);
            driveBackward(0.5, 14);
            strafeRight(0.5, 31);
            turnRight(0.3, 45);

            telemetry.addData("Status", "Task complete");
            telemetry.update();
        }
    }

    private void drive(double lfPower, double rfPower, double lbPower, double rbPower, long duration) throws InterruptedException {
        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftBack.setPower(lbPower);
        rightBack.setPower(rbPower);
        sleep(duration);
        stopDriving();
    }

    private void strafeRight(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * strafeCorrectionFactor;
        double time = correctedDistance / 50 * 1000;
        drive(-speed, speed, speed, -speed, (long) time);
    }

    private void strafeLeft(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * strafeCorrectionFactor;
        double time = correctedDistance / 50 * 1000;
        drive(speed, -speed, -speed, speed, (long) time);
    }

    private void driveForward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * forwardBackwardCorrectionFactor;
        double time = correctedDistance / 50 * 1000;
        drive(-speed, -speed, -speed, -speed, (long) time);
    }

    private void driveBackward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * forwardBackwardCorrectionFactor;
        double time = correctedDistance / 50 * 1000;
        drive(speed, speed, speed, speed, (long) time);
    }

    private void turnLeft(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * angleCorrectionFactor;
        double time = correctedAngle / 90 * 1000;
        drive(speed, -speed, speed, -speed, (long) time);
    }

    private void turnRight(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * angleCorrectionFactor;
        double time = correctedAngle / 90 * 1000;
        drive(-speed, speed, -speed, speed, (long) time);
    }

    private void stopDriving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}