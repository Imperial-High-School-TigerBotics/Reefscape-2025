package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TeamDependentFactors;

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

    @Override
    public void periodic() {
        updatePoseEstimator();

        LimelightHelpers.PoseEstimate epose = null;

        try {
            if (TeamDependentFactors.getAlliance() == Alliance.Red) {
                epose = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
            } else if (TeamDependentFactors.getAlliance() == Alliance.Blue) {
                epose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            }

            if (epose != null 
                && epose.pose != null 
                && epose.tagCount > 0 
                && !Double.isNaN(epose.pose.getX()) 
                && !Double.isNaN(epose.pose.getY()) 
                && epose.timestampSeconds > 0) {

                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                poseEstimator.addVisionMeasurement(epose.pose, epose.timestampSeconds);
            }
        } catch (Exception e) {
            SmartDashboard.putString("Vision Error", e.toString());
        }

        estimatedPosition = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("vision estimate x", estimatedPosition.getX());
        SmartDashboard.putNumber("vision estimate y", estimatedPosition.getY());
    }



    public Pose2d getPoseEstimation() {
        return estimatedPosition == null ? swerve.getPose() : estimatedPosition;
    }

}