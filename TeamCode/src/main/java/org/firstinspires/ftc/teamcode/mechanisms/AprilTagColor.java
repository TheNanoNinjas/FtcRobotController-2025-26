package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Mecanum Color Auto Drive (SMOOTH)")
public class AprilTagColor extends OpMode {

    // LIMELIGHT
    private Limelight3A limelight;

    // MOTORS
    private DcMotor fl_motor, bl_motor, fr_motor, br_motor;

    // ================= TUNING =================
    private static final double STRAFE_KP = 0.012;
    private static final double TURN_KP   = 0.003;

    private static final double MAX_FORWARD = 0.30;
    private static final double MAX_STRAFE  = 0.25;
    private static final double MAX_TURN    = 0.18;

    private static final double TX_DEADBAND = 3.5;
    private static final double TARGET_AREA_STOP = 6.5;

    // Smoothing
    private double filteredTx = 0.0;
    private static final double TX_FILTER_ALPHA = 0.90;

    // Slew rate (anti-jerk)
    private double smoothForward = 0.0;
    private double smoothStrafe  = 0.0;
    private double smoothTurn    = 0.0;
    private static final double SLEW_RATE = 0.04;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(7); // COLOR PIPELINE
        limelight.stop();

        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.REVERSE);

        stopDrive();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        double targetForward = 0.0;
        double targetStrafe  = 0.0;
        double targetTurn    = 0.0;

        if (result != null && result.isValid()) {

            double tx = result.getTx();
            double ta = result.getTa();

            // ===== FILTER TX =====
            filteredTx = TX_FILTER_ALPHA * filteredTx + (1 - TX_FILTER_ALPHA) * tx;

            // ===== STRAFE =====
            if (Math.abs(filteredTx) > TX_DEADBAND) {
                targetStrafe = clamp(filteredTx * STRAFE_KP, -MAX_STRAFE, MAX_STRAFE);
            }

            // ===== SMALL TURN ASSIST =====
            targetTurn = clamp(filteredTx * TURN_KP, -MAX_TURN, MAX_TURN);

            // ===== FORWARD SCALES WITH DISTANCE =====
            if (Math.abs(filteredTx) < 5.0 && ta < TARGET_AREA_STOP) {
                double distanceFactor = 1.0 - (ta / TARGET_AREA_STOP);
                targetForward = clamp(distanceFactor * MAX_FORWARD, 0, MAX_FORWARD);
            }

        }

        // ===== APPLY SLEW RATE (ANTI-JERK) =====
        smoothForward = slew(smoothForward, targetForward, SLEW_RATE);
        smoothStrafe  = slew(smoothStrafe, targetStrafe,  SLEW_RATE);
        smoothTurn    = slew(smoothTurn, targetTurn,    SLEW_RATE);

        setMecanumPower(smoothForward, smoothStrafe, smoothTurn);

        telemetry.addData("TX", filteredTx);
        telemetry.addData("TA", result != null ? result.getTa() : 0);
        telemetry.addData("Fwd", smoothForward);
        telemetry.addData("Str", smoothStrafe);
        telemetry.addData("Turn", smoothTurn);
        telemetry.update();
    }

    // ================= MECANUM =================

    private void setMecanumPower(double forward, double strafe, double turn) {

        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))
        );

        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        fl_motor.setPower(lf);
        fr_motor.setPower(rf);
        bl_motor.setPower(lb);
        br_motor.setPower(rb);
    }

    private double slew(double current, double target, double rate) {
        if (target > current) {
            return Math.min(current + rate, target);
        } else {
            return Math.max(current - rate, target);
        }
    }

    private void stopDrive() {
        fl_motor.setPower(0);
        bl_motor.setPower(0);
        fr_motor.setPower(0);
        br_motor.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
