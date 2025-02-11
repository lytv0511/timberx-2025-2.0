package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "StudioConceptAuto", group = "Autonomous")
public class StudioConceptAuto extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private Servo clawServo, clawArmServo;
    private ThreeWheelOdometryTest odometry;

    int startLeft = odometry.getLeftEncoderTicks();
    int startStrafe = odometry.getStrafeEncoderTicks();

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
            odometry.updatePose();
            clawArmServo.setPosition(0.87);
            sleep(500);

            moveUsingOdometry(0.5, 36, 0);
            moveUsingOdometry(0.5, 0, -54);
            turnUsingOdometry(0.3, -45);
            moveUsingOdometry(0.5, 0, -13);
            clawArmServo.setPosition(0.87);
            clawServo.setPosition(0.4);
            sleep(1000);
            clawArmServo.setPosition(0.17);
            moveUsingOdometry(0.5, 0, 13);
            moveUsingOdometry(0.5, 0, 3);
            turnUsingOdometry(0.3, -45);

            telemetry.addData("Status", "Task complete");
            telemetry.update();
        }
    }

    private void moveUsingOdometry(double speed, int targetLeftTicks, int targetStrafeTicks) {
        int startLeft = odometry.getLeftEncoderTicks();
        int startStrafe = odometry.getStrafeEncoderTicks();

        int targetLeft = startLeft + targetLeftTicks;
        int targetStrafe = startStrafe + targetStrafeTicks;

        while (opModeIsActive() && !hasReachedTarget(targetLeft, targetStrafe)) {
            odometry.updatePose();
            int currentLeft = odometry.getLeftEncoderTicks();
            int currentStrafe = odometry.getStrafeEncoderTicks();

            double drive = (targetLeft - currentLeft) * speed;
            double strafe = (targetStrafe - currentStrafe) * speed;

            setMotorPowers(drive, strafe, 0);

            telemetry.addData("Left Encoder", currentLeft);
            telemetry.addData("Strafe Encoder", currentStrafe);
            telemetry.update();
            sleep(10);
        }
        stopDriving();
    }

    private void turnUsingOdometry(double speed, double angle) {
        double correctedAngle = Math.toRadians(angle * angleCorrectionFactor);
        odometry.updatePose();
        double targetHeading = odometry.getHeading() + correctedAngle;

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
        leftFront.setPower(drive + strafe + turn);
        rightFront.setPower(drive - strafe - turn);
        leftBack.setPower(drive - strafe + turn);
        rightBack.setPower(drive + strafe - turn);
    }

    private boolean hasReachedTarget(int targetLeft, int targetStrafe) {
        return Math.abs(targetLeft - odometry.getLeftEncoderTicks()) < 5 &&
                Math.abs(targetStrafe - odometry.getStrafeEncoderTicks()) < 5;
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
