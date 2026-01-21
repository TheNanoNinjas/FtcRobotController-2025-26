package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AprilTagLimelight extends OpMode {
   private Limelight3A Limelight;

    @Override
    public void init() {
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
Limelight.pipelineSwitch(8);
    }

    @Override
    public void start(){


}
    @Override
    public void loop() {

    }
}
