package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    MecanumDrive drive;
    Shooter shooter;
    Intaker intake;
    Gate gate;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        intake = new Intaker(robot);
        gate = new Gate(robot);

        robot.logHardwareStatus(telemetry);
        robot.displayPortMapping(telemetry);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriving();
            handleShooting();
            handleIntake();
            handleGate();
        }
    }

    private void handleDriving() {
        // Reset IMU heading with A button
        if (gamepad1.a) {
            robot.imu.resetYaw();
        }

        // Field-relative drive (default) or robot-relative (left bumper)
        if (gamepad1.left_bumper) {
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

    private void handleShooting() {
        if (gamepad2.right_bumper) {
            shooter.startShooting();
        } else {
            shooter.stopShooting();
        }
    }

    private void handleIntake() {
        if (gamepad2.left_bumper) {
            intake.startPushing();
        } else {
            intake.stopPushing();
        }
    }

    private void handleGate() {
        if (gamepad2.triangle) {
            gate.openGate();
        } else {
            gate.closeGate();
        }
    }
}