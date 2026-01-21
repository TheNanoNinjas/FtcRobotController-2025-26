package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp
public class ShooterPIDF extends OpMode {

    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;

   public double highVelocity = 2000;

   public double lowVelocity = 1350;

    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;


    double[] stepSizes = {10, 1, 0.1, 0.01, 0.001};

    int stepIndex = 1;

    @Override
public void init(){
leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

rightShooter.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0 , F);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Done");
}

    @Override
    public void loop() {

    }

}
