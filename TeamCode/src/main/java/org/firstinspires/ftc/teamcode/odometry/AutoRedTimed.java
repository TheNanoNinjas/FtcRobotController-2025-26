package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelight;

@Autonomous(name = "Auto Red Alliance Timed Far", group = "Competition")
public class AutoRedTimed extends LinearOpMode {

    private RobotHardware robot;
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher pusher;
    private Intaker intake;
    private AprilTagLimelight tagLimelight;

    private static final double SHOOTER_MIN_RPM = 1550;
    private static final double SHOOTER_MAX_RPM = 1650;

    @Override
    public void runOpMode() {

        // ---------------- INIT ----------------
        robot = new RobotHardware();
        robot.init(hardwareMap);

        tagLimelight = new AprilTagLimelight(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot, tagLimelight);
        pusher = new ArtifactPusher(robot, shooter);
        intake = new Intaker(robot);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // ---------------- AUTON ----------------
        driveForwardTimed(0.3, 400);
        turnRightTimed(0.3, 290);

        launchArtifacts(5000);

        turnLeftTimed(0.3, 300);
        driveForwardTimed(0.3, 1650);

        turnLeftTimed(0.3, 1400);
        sleep(500);

        intake.startPushing();
        driveBackwardTimed(0.35, 2500);

        driveForwardTimed(0.3, 2500);


        turnRightTimed(0.3, 1200);
        intake.stopPushing();

        driveBackwardTimed(0.25, 1100);

        sleep(1000);
        driveForwardTimed(0.3, 300);

        turnRightTimed(0.3, 280);
        launchArtifacts(5000);

        driveForwardTimed(0.3, 2000);

        // ---------------- END ----------------
        stopAll();
    }

    // ================= SHOOTING =================

    private void launchArtifacts(long totalTimeMs) {

        shooter.startShootingAutoFar();

        while (opModeIsActive() && !shooter.isFarShotReady()) {
            idle();
        }

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < totalTimeMs) {

            pusher.startArtifactPushing();
            intake.startPushing();
            sleep(150);   // feed time

            pusher.stopPushing();
            intake.stopPushing();
            sleep(1500);
        }

        shooter.stopShooting();
    }




    // ================= DRIVE HELPERS =================

    private void driveForwardTimed(double power, long timeMs) {
        setDrivePower(power, power, power, power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void driveBackwardTimed(double power, long timeMs) {
        setDrivePower(-power, -power, -power, -power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void turnRightTimed(double power, long timeMs) {
        setDrivePower(power, -power, power, -power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void turnLeftTimed(double power, long timeMs) {
        setDrivePower(-power, power, -power, power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        robot.fl_motor.setPower(fl);
        robot.fr_motor.setPower(fr);
        robot.bl_motor.setPower(bl);
        robot.br_motor.setPower(br);
    }

    private void stopAll() {
        robot.stopAllMotors();
        shooter.stopShooting();
        intake.stopPushing();
        pusher.stopPushing();
    }
}
