package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.net.PortForwarder;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    public String name;

    /*private double yUpperBound = 0;
    private double yDownBound = 480;
    private double xLeftBound = 0;
    private double xRightBound = 640;*/

    private int deadBand = 10;

    public Limelight(String name) {
        this.name = name;
    }
    public Limelight() {
        this.name = "limelight";
    }

    public double[] percentPosition(double[] tagInfo) { // value from 0 (top/left) to 1 (bottom/right)
        if (tagInfo == null) {
            return null;
        }
        return new double[] {
            (tagInfo[1] + 27.0) / 54.0,
            (tagInfo[2] + 20.5) / 41.0
        };
    }

    public void updateValues() {

        /*LimelightHelpers.LimelightTarget_Fiducial[] targets = LimelightHelpers.getLatestResults(name).targets_Fiducials;
        
        double[] ids = new double[32];
        for (var target : targets) {
            ids. = target.fiducialID;
            if (ids[i] == Constants.TeamDependentFactors.middleAprilTagId) {
                var pos = percentPosition(targets[i]);
                SmartDashboard.putNumber("yyyyyy", pos[1]);
            }
        }
        SmartDashboard.putNumberArray("ids", ids);*/
    }

    public double getDeadBandedPosition() {
        double[] ppos = percentPosition(getTarget(Constants.TeamDependentFactors.middleAprilTagId));
        if (ppos == null) {
            return 2;
        }
        return -(ppos[0] * 2 - 1) * Constants.SpeedScaleFactors.autoTurnSpeed; // value between -1 and 1, slowing it down by autoTurnSpeed
    }

    public void portForward() { // we dont need this but dont delete it in case we actually do need this
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
    }

    public void stopAprilTagDetector() {}

    public double[] getTarget(int id) {
        //var lresults = LimelightHelpers.getLatestResults(name);
        //System.out.println(lresults.targets_Fiducials.length);

        var targets = NetworkTableInstance.getDefault().getTable(name).getEntry("rawfiducials").getDoubleArray(new double[] {-1, 0, 0, 0, 0, 0, 0});
        for (int i = 0; i < targets.length; i += 7) {
            if (targets[i] == id) {
                return new double[] {
                    targets[i],
                    targets[i + 1],
                    targets[i + 2]
                };
            }
        }
        return null;
    }


    /*public PoseEstimate estimatePose() {

        LimelightHelpers.SetRobotOrientation(name, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        PoseEstimate poseEstimateNew = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        int apriltagcount = poseEstimate.tagCount;
        SmartDashboard.putNumber("apriltag count", apriltagcount);

        if (apriltagcount > 0 && fieldBoundary.isPoseWithinArea(poseEstimate.pose)) {
            if(poseEstimate.avgTagDist < 3.6576) {
                confidence = 0.5;
            } else {
                // If more than 12 ft away use MegaTag 2 use MT if less than 12
                poseEstimate = poseEstimateNew;
                confidence = 0.7;
            }
        }

        return poseEstimate;
    }*/
}