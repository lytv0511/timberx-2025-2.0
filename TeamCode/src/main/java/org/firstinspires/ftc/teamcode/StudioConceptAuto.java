package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "StudioConceptAuto", group = "Autonomous")
public class StudioConceptAuto extends LinearOpMode {
    // Declare four drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Declare servos
    private Servo clawServo, clawArmServo;

    // Declare odometry system
    private ThreeWheelOdometryTest odometry;

    // Correction factors
    private static final double strafeCorrectionFactor = 10.0 / 9.0;
    private static final double forwardBackwardCorrectionFactor = 10.0 / 11.0;
    private static final double angleCorrectionFactor = 90.0 / 45.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set motor zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize odometry
        odometry = new ThreeWheelOdometryTest(hardwareMap);

        telemetry.addData("Status", "Waiting for start command...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            odometry.updatePose();

            // Claw arm initialization
            clawArmServo.setPosition(0.87);
            sleep(500);

            // Autonomous movements using odometry
            moveUsingOdometry(0.5, 36, 0);
            moveUsingOdometry(0.5, 0, -54);
            turnUsingOdometry(0.3, -45);
            moveUsingOdometry(0.5, 0, -13);

            // Claw operations
            clawArmServo.setPosition(0.87);
            clawServo.setPosition(0.4);
            sleep(1000);
            clawArmServo.setPosition(0.17);
            moveUsingOdometry(0.5, 0, 13);
            moveUsingOdometry(0.5, 0, 3);
            turnUsingOdometry(0.3, -45);

            // Second placement
            clawArmServo.setPosition(0.07);
            sleep(1000);
            moveUsingOdometry(0.5, 0, 14);
            clawServo.setPosition(0.7);
            sleep(1000);
            clawArmServo.setPosition(0.88);
            sleep(1000);
            moveUsingOdometry(0.5, 0, -14);
            moveUsingOdometry(0.5, 0, -3);
            turnUsingOdometry(0.3, 45);

            // Third placement
            moveUsingOdometry(0.5, 0, -13);
            clawArmServo.setPosition(0.87);
            clawServo.setPosition(0.4);
            sleep(1000);
            clawArmServo.setPosition(0.17);
            moveUsingOdometry(0.5, 0, 13);
            moveUsingOdometry(0.5, 0, 3);
            turnUsingOdometry(0.3, -45);

            // Move to final position
            moveUsingOdometry(0.5, 31, 0);
            clawArmServo.setPosition(0.07);
            sleep(1000);
            moveUsingOdometry(0.5, 0, 14);
            clawServo.setPosition(0.7);
            sleep(1000);
            clawArmServo.setPosition(0.87);
            sleep(1000);
            moveUsingOdometry(0.5, 0, -14);
            moveUsingOdometry(0.5, -31, 0);
            turnUsingOdometry(0.3, 45);

            telemetry.addData("Status", "Task complete");
            telemetry.update();
        }
    }

    // Drive functions using odometry
    private void moveUsingOdometry(double speed, double strafeDistance, double forwardDistance) throws InterruptedException {
        double correctedStrafeDistance = strafeDistance * strafeCorrectionFactor;
        double correctedForwardDistance = forwardDistance * forwardBackwardCorrectionFactor;

        odometry.updatePose();
        double targetX = odometry.getX() + correctedStrafeDistance;
        double targetY = odometry.getY() + correctedForwardDistance;

        while (opModeIsActive() && !hasReachedTarget(targetX, targetY)) {
            odometry.updatePose();

            double errorX = targetX - odometry.getX();
            double errorY = targetY - odometry.getY();

            double drive = errorY * speed;
            double strafe = errorX * speed;

            setMotorPowers(drive, strafe, 0);

            telemetry.addData("X", odometry.getX());
            telemetry.addData("Y", odometry.getY());
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();
        }

        stopDriving();
    }

    private void turnUsingOdometry(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * angleCorrectionFactor;

        odometry.updatePose();
        double targetHeading = odometry.getHeading() + Math.toRadians(correctedAngle);

        while (opModeIsActive() && !hasReachedHeading(targetHeading)) {
            odometry.updatePose();

            double errorHeading = targetHeading - odometry.getHeading();
            double turn = errorHeading * speed;

            setMotorPowers(0, 0, turn);

            telemetry.addData("Heading", odometry.getHeading());
            telemetry.addData("Target Heading", targetHeading);
            telemetry.update();
        }

        stopDriving();
    }

    private void setMotorPowers(double drive, double strafe, double turn) {
        double lfPower = drive + strafe + turn;
        double rfPower = drive - strafe - turn;
        double lbPower = drive - strafe + turn;
        double rbPower = drive + strafe - turn;

        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftBack.setPower(lbPower);
        rightBack.setPower(rbPower);
    }

    private boolean hasReachedTarget(double targetX, double targetY) {
        return Math.abs(targetX - odometry.getX()) < 0.5 && Math.abs(targetY - odometry.getY()) < 0.5;
    }

    private boolean hasReachedHeading(double targetHeading) {
        return Math.abs(targetHeading - odometry.getHeading()) < Math.toRadians(2.5);
    }

    private void stopDriving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
