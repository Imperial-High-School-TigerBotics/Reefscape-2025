package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Primary Swerve Subsystem responsible for:
 *   - Driving the swerve modules
 *   - Holding a single SwerveDrivePoseEstimator for fused wheel+vision odometry
 *   - Maintaining the "latest" robot pose
 */
public class Swerve extends SubsystemBase {
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;

    // We'll use a SwerveDrivePoseEstimator instead of a plain odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    private boolean autonMovingEnabled = true;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.SwerveStartHeading);

        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Create the pose estimator. Typically you can add standard-deviation
        // parameters; or just use the simpler version:
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );
    }

    /**
     * Main drive method (usually used by auto or by a Holonomic PID).
     * If autonMovingEnabled == false, we zero out motion.
     */
    public void drive(ChassisSpeeds speeds) {
        if (!autonMovingEnabled) {
            speeds = new ChassisSpeeds();
        }

        SwerveModuleState[] moduleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates, 
            Constants.Swerve.maxSpeed
        );

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(moduleStates[mod.moduleNumber], true);
        }
    }

    /**
     * Teleop drive using translation + rotation in either field or robot relative.
     */
    public void drive(Translation2d translation, double rot, boolean fieldRelative, boolean isOpenLoop) {
        if (!autonMovingEnabled) {
            translation = new Translation2d();
            rot = 0;
        }

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rot,
                        getGyroYaw()
                    )
                  : new ChassisSpeeds(translation.getX(), translation.getY(), rot)
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Returns the current ChassisSpeeds, derived from module states (and thus real velocity).
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Rotation2d getGyroYaw() {
        // Pigeon2 'getRotation2d()' returns a standard CCW-positive angle in WPILib convention
        return gyro.getRotation2d();
    }

    /**
     * The single, fused best-guess of the robot pose:
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the estimated pose to a given pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(
            getGyroYaw(),
            getModulePositions(),
            pose
        );
    }

    /**
     * If you want to manually update heading without changing XY, or forcibly set the heading:
     */
    public void setHeading(Rotation2d newHeading) {
        Pose2d current = getPose();
        Pose2d newPose = new Pose2d(current.getTranslation(), newHeading);
        setPose(newPose);
    }

    /**
     * Zero heading
     */
    public void zeroHeading() {
        // Force the pose estimator to treat the current orientation as 0 degrees
        setHeading(new Rotation2d());
    }

    /**
     * Example command to flip heading by 180
     */
    public InstantCommand flipHeading() {
        return new InstantCommand(() -> {
            setHeading(getPose().getRotation().plus(Rotation2d.fromDegrees(180)));
        });
    }

    /**
     * Add a vision measurement from your Limelight (or other sensor).  Typically
     * called from Vision subsystem's periodic if a detection is valid.
     */
    public void addVisionMeasurement(Pose2d visionEstimatedPose, double timeSeconds) {
        poseEstimator.addVisionMeasurement(visionEstimatedPose, timeSeconds);
    }

    public void enableAutonMoving() {
        autonMovingEnabled = true;
    }

    public void disableAutonMoving() {
        autonMovingEnabled = false;
    }

    @Override
    public void periodic() {
        // 1) Update pose estimator from wheel odometry each loop
        poseEstimator.update(
            getGyroYaw(),
            getModulePositions()
        );

        // Just for debugging:
        Pose2d currentPose = getPose();
        SmartDashboard.putNumber("Swerve Pose X", currentPose.getX());
        SmartDashboard.putNumber("Swerve Pose Y", currentPose.getY());
        SmartDashboard.putNumber("Swerve Pose Heading", currentPose.getRotation().getDegrees());
    }

    /**
     * If you need direct access to all 8 TalonFX objects.
     */
    public TalonFX[] getTalons() {
        TalonFX[] talons = new TalonFX[8];
        for (int i = 0; i < 4; i++) {
            talons[i * 2] = mSwerveMods[i].getTalons()[0];
            talons[i * 2 + 1] = mSwerveMods[i].getTalons()[1];
        }
        return talons;
    }

    /**
     * Resets the absolute encoders on each swerve module, if needed.
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }
}
