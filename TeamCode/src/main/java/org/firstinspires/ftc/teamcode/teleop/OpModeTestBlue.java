package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelightBlue;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusherBlue;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterBlue;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name = "OpMode Test Blue")
public class OpModeTestBlue extends OpMode {

    RobotHardware robot = new RobotHardware();
    MecanumDrive drive;
    ShooterBlue shooterBlue;
    Intaker intake;
    ArtifactPusherBlue artifactPusher;
    AprilTagLimelightBlue tagLimelightBlue;

    boolean shooterWasReady = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);

        tagLimelightBlue = new AprilTagLimelightBlue(hardwareMap);

        shooterBlue = new ShooterBlue(robot, tagLimelightBlue);

        intake = new Intaker(robot);
        artifactPusher = new ArtifactPusherBlue(robot, shooterBlue);

        tagLimelightBlue.start();
    }

    @Override
    public void loop() {
        handleDriving();
        handleShooting();
        handleFeeding();
        tagLimelightBlue.update();
        updateTelemetry();
    }

    private void handleDriving() {
        if (gamepad1.cross) {
            robot.imu.resetYaw();
        }

        if (gamepad1.right_bumper) {
            drive.driveFieldRelative(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    robot.imu
            );
        } else {
            drive.mecanumDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }
    }

    private void handleShooting() {
        boolean wantsCloseShot = gamepad2.right_bumper;
        boolean wantsFarShot = gamepad2.left_bumper;

        if (wantsCloseShot) {
            shooterBlue.shootTagsClose();
        } else if (wantsFarShot) {
            shooterBlue.shootTagsFar();
        }else if(gamepad2.triangle){
            shooterBlue.startShootingFar();
        }
        else if (gamepad2.circle){
            shooterBlue.startShootingClose();
        }
        else if (gamepad2.square || gamepad1.left_bumper) {
            shooterBlue.manualIntakeShooter();
            shooterWasReady = false;
            return;
        } else {
            shooterBlue.stopShooting();
            shooterWasReady = false;
            return;
        }

        boolean shooterReady =
                (wantsCloseShot && shooterBlue.isCloseShotReady()) ||
                        (wantsFarShot && shooterBlue.isFarShotReady());

        if (shooterReady && !shooterWasReady) {
            gamepad2.rumble(300);
        }

        shooterWasReady = shooterReady;
    }

    private void handleFeeding() {
        boolean wantsCloseShot = gamepad2.right_bumper;
        boolean wantsFarShot = gamepad2.left_bumper;

        if (gamepad2.cross) {
            if (wantsCloseShot) {
                artifactPusher.driverPushClose(true);
            } else if (wantsFarShot) {
                artifactPusher.driverPushFar(true);
            }
            intake.startPushing();
        }
        else if (gamepad2.right_stick_button || gamepad1.right_trigger > 0) {
            artifactPusher.reverseArtifacts();
            intake.reversePush();
        }
        else if (gamepad2.left_trigger > 0) {
            artifactPusher.reverseArtifacts();
        }
        else if (gamepad1.right_bumper){
            artifactPusher.startWheelIntake();
        }
        else if (gamepad2.dpad_up || gamepad1.left_bumper){
            intake.startPushing();
        }
        else if (gamepad2.right_trigger > 0){
            artifactPusher.startArtifactPushing();
        }
        else {
            artifactPusher.stopPushing();
            intake.stopPushing();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Left Shooter Vel", shooterBlue.getLeftVelocity());
        telemetry.addData("Right Shooter Vel", shooterBlue.getRightVelocity());
        telemetry.addData("Distance Inches", tagLimelightBlue.getDistanceInches());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}
