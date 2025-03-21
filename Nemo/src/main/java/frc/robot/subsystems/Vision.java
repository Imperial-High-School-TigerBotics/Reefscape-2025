package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    private Swerve swerve;
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d estimatedPosition;
    
    public Vision(Swerve swerve) {
        LimelightHelpers.SetRobotOrientation("", swerve.gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        this.swerve = swerve;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            swerve.gyro.getRotation2d(),
            swerve.getModulePositions(),
            new Pose2d(0, 0, new Rotation2d(0))
        );
    }

    public void updatePoseEstimator() {
        poseEstimator.update(swerve.gyro.getRotation2d(), swerve.getModulePositions());
    }

    public Pose2d getPoseEstimation() {
        return estimatedPosition == null ? swerve.getPose() : estimatedPosition;
    }
    
    @Override
    public void periodic() {

        updatePoseEstimator();

        LimelightHelpers.PoseEstimate epose;

        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            epose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
        } else {
            epose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        }

        if (epose.tagCount > 0 && epose.pose != null) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            poseEstimator.addVisionMeasurement(epose.pose, epose.timestampSeconds);
            estimatedPosition = poseEstimator.getEstimatedPosition();
        }

        SmartDashboard.putNumber("vision estimatex", estimatedPosition.getX());
        SmartDashboard.putNumber("vision estimatey", estimatedPosition.getY());
    }

}