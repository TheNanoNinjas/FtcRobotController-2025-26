package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name = "Main TeleOp")
@Disabled
public class MainTeleOp extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    MecanumDrive drive;
    Shooter shooter;
    Intaker intake;
    ArtifactPusher artifactPusher;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        intake = new Intaker(robot);
     //   artifactPusher = new ArtifactPusher(robot);

        robot.logHardwareStatus(telemetry);
        robot.displayPortMapping(telemetry);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
           // handleShooting();
            handleIntake();
            handlePush();
            launchArtifacts();
            releaseArtifacts();

        }
    }

    private void handleDriving() {
        // Reset IMU heading with A button
        if (gamepad1.cross) {
            robot.imu.resetYaw();
        }

        // Field-relative drive (default) or robot-relative (right bumper)
        if (gamepad1.right_bumper) {
            // Robot-relative drive
            drive.mecanumDrive(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        } else {
            // Field-relative drive
            drive.driveFieldRelative(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.imu);
        }
    }

 /*   private void handleShooting() {
        if (gamepad2.right_bumper) {
            shooter.startShootingFar();
        } else {
            shooter.stopShooting();
        }
    }*/

    private void handleIntake() {
        if (gamepad1.left_bumper) {
            intake.startPushing();
            intake.startPushing();
        }else if(gamepad2.cross){
            intake.startPushing();
        } else if(gamepad2.right_stick_button){
            intake.reversePush();
        }
        else {
            intake.stopPushing();
        }
    }


    private void handlePush(){
        if (gamepad2.cross){
            artifactPusher.startArtifactPushing();
           // intake.startPushing();
        }else {
            artifactPusher.stopPushing();
           // intake.stopPushing();
        }
    }
    private void launchArtifacts() {
        //Square is for Close range shooting
        if (gamepad2.right_bumper) {
            shooter.startShootingClose();
           // sleep(1000);
            //intake.startPushing();
            //artifactPusher.startWheel();
        }
        //Circle is for Far range shooting
        else if(gamepad2.left_bumper){
            shooter.startShootingFar();
         //   sleep(1000);
            //intake.startPushing();
            //artifactPusher.startWheel();

        }else if(gamepad2.square){
            shooter.manualIntakeShooter();
        }
        else {
            shooter.stopShooting();
            //intake.stopPushing();
            //artifactPusher.stopPushing();
        }
    }


    private void releaseArtifacts(){
        if (gamepad2.right_stick_button){
            intake.reversePush();
            artifactPusher.reverseArtifacts();
        } else {
          //  intake.stopPushing();
            artifactPusher.stopPushing();
        }
    }
}