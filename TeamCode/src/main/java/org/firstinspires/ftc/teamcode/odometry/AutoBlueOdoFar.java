package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

@Autonomous(name = "Auto Blue ODO Far ", group = "Competition")
public class AutoBlueOdoFar extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher pusher;
    private Intaker intake;

    // ===== TUNING CONSTANTS =====
    private static final double KP_POS = 0.004;
    private static final double KP_HEADING = 0.015;

    private static final double MAX_POWER = 0.35;
    private static final double POSITION_TOLERANCE = 5; // mm
    private static final double HEADING_TOLERANCE = 1.0; // degrees

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        pusher = new ArtifactPusher(robot);
        intake = new Intaker(robot);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        robot.resetOdo();

        if (opModeIsActive()) {
            runAuto();
        }
    }

    private void runAuto() throws InterruptedException {
// Shoot
        moveToY(225, 0);
        turnToHeading(15);
        launchArtifacts();

// Drive forward
        turnToHeading(20);
        moveToY(850, 0);

// Move backward relative to robot
        turnToHeading(270);
        moveToX(-900, 270);

// Return
        moveToX(0, 270);
        turnToHeading(0);
        moveToY(0, 0);

        moveToY(225,0);

        turnToHeading(15);

        launchArtifacts();

    }



    private void moveToY(double targetY, double heading) {
        long startTime = System.currentTimeMillis();
        long timeout = 6000;

        while (opModeIsActive()) {

            if (System.currentTimeMillis() - startTime > timeout) break;

            Pose2D pose = robot.getOdoPosition();
            double y = pose.getY(DistanceUnit.MM);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorY = targetY - y;
            if (Math.abs(errorY) < POSITION_TOLERANCE) break;

            double drivePower = errorY * KP_POS;
            drivePower = clip(drivePower);

            double headingError = normalizeAngle(heading - currentHeading);
            double correction = headingError * KP_HEADING;

            mecanumDrive(drivePower, 0, correction);

            telemetry.addData("Target Y", targetY);
            telemetry.addData("Current Y", y);
            telemetry.addData("Error Y", errorY);
            telemetry.update();
        }

        drive.stop();
    }

    private void moveToX(double targetX, double heading) {
        long startTime = System.currentTimeMillis();
        long timeout = 6000;

        while (opModeIsActive()) {

            if (System.currentTimeMillis() - startTime > timeout) break;

            Pose2D pose = robot.getOdoPosition();
            double x = pose.getX(DistanceUnit.MM);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - x;
            if (Math.abs(errorX) < POSITION_TOLERANCE) break;

            double strafePower = clip(errorX * KP_POS);

            double headingError = normalizeAngle(heading - currentHeading);

            double correction = clip(headingError * 0.003);

            mecanumDrive(0, strafePower, correction);

            telemetry.addData("Target X", targetX);
            telemetry.addData("Current X", x);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }

        drive.stop();
    }


    private void turnToHeading(double target) {

        long startTime = System.currentTimeMillis();
        long timeout = 4000;

        while (opModeIsActive()) {

            if (System.currentTimeMillis() - startTime > timeout) break;

            double current = robot.getOdoHeading(AngleUnit.DEGREES);

            // Normalize BOTH angles first
            double error = normalizeAngle(target) - normalizeAngle(current);
            error = normalizeAngle(error);

            if (Math.abs(error) < HEADING_TOLERANCE) break;

            double turnPower = clip(error * 0.012);

            mecanumDrive(0, 0, turnPower);

            telemetry.addData("Target", target);
            telemetry.addData("Heading", current);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        drive.stop();
    }

    private void mecanumDrive(double forward, double strafe, double turn) {
        double fl = forward + strafe - turn;
        double fr = forward - strafe + turn;
        double bl = forward - strafe - turn;
        double br = forward + strafe + turn;

        robot.fl_motor.setPower(clip(fl));
        robot.fr_motor.setPower(clip(fr));
        robot.bl_motor.setPower(clip(bl));
        robot.br_motor.setPower(clip(br));
    }

    private double clip(double val) {
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, val));
    }

    private double normalizeAngle(double angle) {
        return ((angle + 180) % 360) - 180;
    }
    private void strafeToX(double targetXMM, double basePower) {

        Pose2D startPos = robot.getOdoPosition();
        double startHeading = startPos.getHeading(AngleUnit.DEGREES);

        double currentX = startPos.getX(DistanceUnit.MM);
        double error = targetXMM - currentX;
        double direction = Math.signum(error);

        telemetry.addLine("Strafe Started");
        telemetry.update();

        while (opModeIsActive()) {

            Pose2D pos = robot.getOdoPosition();

            currentX = pos.getX(DistanceUnit.MM);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);
            error = targetXMM - currentX;

            // Stop when close enough
            if (Math.abs(error) < 0.5) break;

            // Calculate heading error
            double headingError = ((startHeading - currentHeading + 180) % 360) - 180;


            double correction = 0.02 * headingError;  // tweak 0.02 if needed

            // Base strafe power
            double strafePower = direction * Math.abs(basePower);

            // Apply heading correction to each side
            double flPower = strafePower - correction;
            double blPower = -strafePower - correction;
            double frPower = -strafePower + correction;
            double brPower = strafePower + correction;

            robot.fl_motor.setPower(-flPower);
            robot.bl_motor.setPower(blPower);
            robot.fr_motor.setPower(frPower);
            robot.br_motor.setPower(-brPower);

            telemetry.addData("Target X (in)", targetXMM);
            telemetry.addData("Current X (in)", currentX);
            telemetry.addData("Remaining Distance (in)", "%.2f", error);
            telemetry.addData("Heading", "%.2f", currentHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            sleep(20);
        }

        robot.stopAllMotors();
        telemetry.addLine("Strafe Completed");
        telemetry.update();
    }



    private void launchArtifacts() throws InterruptedException {
        shooter.startShootingFar();
        sleep(1500);

        pusher.startWheel();
        sleep(800);

        intake.startPushingAuto(700);
        sleep(600);

        shooter.stopShooting();
        intake.stopPushing();
        pusher.stopPushing();
    }
}
