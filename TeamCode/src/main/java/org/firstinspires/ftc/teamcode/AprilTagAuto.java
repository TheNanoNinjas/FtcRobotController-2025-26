/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagAuto extends OpMode {
    AprilTags aprilTags = new AprilTags();

    @Override
    public void init() {
        aprilTags.init(hardwareMap, telemetry);
    }


    @Override
    public void loop() {
        aprilTags.update();
        AprilTagDetection id20 = aprilTags.getTagBySpecificID(20);
        telemetry.addData("id20 String", id20.toString());
    }
}
*/