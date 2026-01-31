    package org.firstinspires.ftc.teamcode.mechanisms;

    import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
    import org.firstinspires.ftc.teamcode.util.RobotHardware;

    import com.qualcomm.hardware.limelightvision.LLResult;
    import com.qualcomm.hardware.limelightvision.Limelight3A;
    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.IMU;

    import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
    @Autonomous(name = "Auto April Tags", group = "Competition")
    public class AprilTagLimelight extends OpMode {
       private Limelight3A Limelight3A;

       private IMU imu;

        @Override
        public void init() {
            Limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
    Limelight3A.pipelineSwitch(8);
            imu = hardwareMap.get(IMU.class, "imu");

            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP 
                    )
            );
            imu.initialize(parameters);

        }

        @Override
        public void start(){
    Limelight3A.start();

    }
        @Override
        public void loop() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            Limelight3A.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = Limelight3A.getLatestResult();

            if(llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                double distance = getDistanceFromTags(llResult.getTa());
                telemetry.addData("Calculated Distance", distance);
                telemetry.addData("Target x",llResult.getTx());
                telemetry.addData("Target y", llResult.getTy());
                telemetry.addData("Target area", llResult.getTa());
                telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                telemetry.addData("Botpose", botPose.toString());
            }
        }
        public double getDistanceFromTags(double ta){
double scale = 30665.95;
double distance = (scale/ta);
return distance;
        }

    }
