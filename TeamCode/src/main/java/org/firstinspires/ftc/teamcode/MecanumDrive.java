package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Relative Mecanum Drive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    // Motors
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;

    // IMU
    private IMU imu;

    @Override
    public void runOpMode() {
        // ---- INIT HARDWARE ----
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        // Reverse left side so all wheels go forward with positive power
        bl_motor.setDirection(DcMotor.Direction.REVERSE);
        fl_motor.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders (optional, but fine)
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // Set this to match your hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for start
        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        waitForStart();

        // ---- MAIN LOOP ----
        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper for robot-relative drive");
            telemetry.addLine("Left stick = direction, Right stick X = turn");

            // Reset yaw
            if (gamepad1.cross) {
                imu.resetYaw();
            }

            double forward = -gamepad1.left_stick_y; // forward/back
            double right   =  gamepad1.left_stick_x; // strafe
            double rotate  =  gamepad1.right_stick_x; // turn

            if (gamepad1.left_bumper) {
                // Robot-centric drive
                drive(forward, right, rotate);
            } else {
                // Field-centric drive
                driveFieldRelative(forward, right, rotate);
            }

            telemetry.update();
        }
    }

    // Field-relative drive: translate joystick based on robot heading
    private void driveFieldRelative(double forward, double right, double rotate) {
        // Convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Subtract robot heading
        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        // Back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        // Drive robot-relative with adjusted values
        drive(newForward, newRight, rotate);
    }

    // Robot-relative mecanum drive
    private void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 0.5;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        double maxSpeed = 0.5; // change < 1.0 to slow robot down

        fl_motor.setPower(maxSpeed * (frontLeftPower  / maxPower));
        fr_motor.setPower(maxSpeed * (frontRightPower / maxPower));
        bl_motor.setPower(maxSpeed * (backLeftPower   / maxPower));
        br_motor.setPower(maxSpeed * (backRightPower  / maxPower));
    }
}
