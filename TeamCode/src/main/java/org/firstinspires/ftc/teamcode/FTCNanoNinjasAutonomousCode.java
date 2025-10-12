package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Arnav+MahithAutonomous", group="Linear Opmode")
public class FTCNanoNinjasAutonomousCode extends LinearOpMode {

    private DcMotor fr_motor, fl_motor, br_motor, bl_motor;
    private DcMotor rightShooter, leftShooter, intakeMotor;
    private CRServo rampServo;
    private DistanceSensor distanceSensor;
    private Rev2mDistanceSensor sensorTimeOfFlight;

    /*private DcMotor fr_motor, fl_motor, br_motor, bl_motor;
    private DcMotor rightShooter, leftShooter, intakeMotor;
    private CRServo rampServo;
    private DistanceSensor distanceSensor;
    private Rev2mDistanceSensor sensorTimeOfFlight;*/
    @Override
    public void runOpMode() {

        // Hardware mapping
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        leftShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rampServo = hardwareMap.get(CRServo.class, "rampServo");

        // Distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;

        // Motor directions
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Click START to begin Autonomous");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // ✅ Drive forward until robot is within 20 cm of an object
            while (opModeIsActive() &&
                    sensorTimeOfFlight.getDistance(DistanceUnit.CM) > 35.0) {

                fr_motor.setPower(0.3);
                fl_motor.setPower(0.3);
                br_motor.setPower(0.3);
                bl_motor.setPower(0.3);

                double distCM = sensorTimeOfFlight.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance (cm)", "%.2f", distCM);
                telemetry.update();
            }

            // Stop when target distance is reached
            stopMotors();
            telemetry.addLine("Target distance reached! Robot stopped.");
            telemetry.update();

            // ✅ Add your other autonomous actions here:
            // Example:
            // driveForward(0.2, 2500);
            // sleep(1000);
            // turnRight(0.2, 2000);
            // stopMotors();
        }
    }

    // ----------- Helper Methods --------------

    private void driveForward(double power, long timeMs) {
        fr_motor.setPower(power);
        fl_motor.setPower(power);
        br_motor.setPower(power);
        bl_motor.setPower(power);
        sleep(timeMs);
        stopMotors();
    }

    private void turnRight(double power, long timeMs) {
        fr_motor.setPower(-power);
        br_motor.setPower(-power);
        fl_motor.setPower(power);
        bl_motor.setPower(power);
        sleep(timeMs);
        stopMotors();
    }

    private void turnLeft(double power, long timeMs) {
        fr_motor.setPower(power);
        br_motor.setPower(power);
        fl_motor.setPower(-power);
        bl_motor.setPower(-power);
        sleep(timeMs);
        stopMotors();
    }

    private void stopMotors() {
        fr_motor.setPower(0);
        fl_motor.setPower(0);
        br_motor.setPower(0);
        bl_motor.setPower(0);
    }
}
