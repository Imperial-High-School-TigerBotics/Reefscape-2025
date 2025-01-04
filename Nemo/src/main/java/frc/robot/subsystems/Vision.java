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
        
        var epose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        poseEstimator.addVisionMeasurement(epose.pose, epose.timestampSeconds);

        SmartDashboard.putNumber("vision estimatex", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("vision estimatey", poseEstimator.getEstimatedPosition().getY());
        //SmartDashboard.putNumber("vision !!!", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    }

}