package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.HogbackAI;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterAI;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name = "OpMode AI")
public class OpModeAI extends OpMode {

    RobotHardware robot = new RobotHardware();
    MecanumDrive drive;
    ShooterAI shooter;
    Intaker intake;
    HogbackAI artifactPusher;

    boolean shooterWasReady = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new ShooterAI(robot);
        intake = new Intaker(robot);
        artifactPusher = new HogbackAI(robot, shooter);
    }

    @Override
    public void loop() {
        handleDriving();
        handleShooting();
        shooter.update();          // REQUIRED
        handleFeeding();
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
            shooter.startShootingClose();
        } else if (wantsFarShot) {
            shooter.startShootingFar();
        } else if (gamepad2.square) {   // FIX: no button conflict
            shooter.manualIntakeShooter();
            shooterWasReady = false;
            return;
        }
        // FIX: NO automatic stopShooting()
    }

    private void handleFeeding() {
        // FIX: use ONLY gated AI feeding
        artifactPusher.feedOneWhenReady(gamepad2.cross);

        if (gamepad2.right_stick_button || gamepad1.right_trigger > 0) {
            artifactPusher.reverseArtifacts();
            intake.reversePush();
        }
        else if (gamepad2.left_trigger > 0) {
            artifactPusher.reverseArtifacts();
        }
        else if (gamepad1.right_bumper){
            artifactPusher.startWheelIntake();
        }
        else if (gamepad2.dpad_up){
            intake.startPushing();
        }
        else if (gamepad1.left_bumper){
            intake.startPushing();
        }
        else {
            artifactPusher.stopPushing();
            intake.stopPushing();
        }
    }

    private void updateTelemetry() {
        Pose2D pos = robot.getOdoPosition();
        telemetry.addData("Left Shooter Vel", shooter.getLeftVelocity());
        telemetry.addData("Right Shooter Vel", shooter.getRightVelocity());
        telemetry.addData("Shooter Ready", shooter.isReadyToFire());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}
