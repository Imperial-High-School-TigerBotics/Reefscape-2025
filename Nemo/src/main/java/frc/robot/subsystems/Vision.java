package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Vision subsystem, periodically pulls the robot’s estimated pose from Limelight,
 * then feeds that measurement into the Swerve subsystem’s SwerveDrivePoseEstimator.
 */
public class Vision extends SubsystemBase {

    private final Swerve swerve;
    private final String limelightName = "limelight"; // change if needed

    public Vision(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        // 1) Check alliance color
        // 2) Retrieve the MegaTag2 pose estimate from either the Red or Blue “botpose_orb”
        //    coordinate system

        LimelightHelpers.PoseEstimate epose;

        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            epose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        } else {
            epose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        }

        // 3) If we got a valid estimate (tagCount > 0, etc.), feed it to our Swerve estimator
        if (epose != null && epose.tagCount > 0) {
            // Add the vision measurement to the swerve’s pose estimator:
            swerve.addVisionMeasurement(epose.pose, epose.timestampSeconds);

            // Debug printouts:
            SmartDashboard.putNumber("Vision Alliance Pose X", epose.pose.getX());
            SmartDashboard.putNumber("Vision Alliance Pose Y", epose.pose.getY());
            SmartDashboard.putNumber("Vision Alliance Heading", epose.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision TagCount", epose.tagCount);
        }
    }

    /**
     * Optional convenience if other code wants the latest “fused” robot pose
     * from the Swerve subsystem’s estimator.
     */
    public Pose2d getEstimatedGlobalPose() {
        return swerve.getPose();
    }
}
