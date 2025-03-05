package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public boolean autonMovingEnabled;
    public PathPlannerAuto a1;
    private Rotation2d lastKnownTagHeading;
    private Rotation2d originalHeading;

    public Swerve(){
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.SwerveStartHeading);
    
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
            };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        autonMovingEnabled = true;

        lastKnownTagHeading = new Rotation2d(); 
        originalHeading = new Rotation2d();
        }
                    
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(ChassisSpeeds speeds) {

        if (!autonMovingEnabled) {
            speeds = new ChassisSpeeds();
        }
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public TalonFX[] getTalons() {
        TalonFX[] talons = new TalonFX[8];
        for (int i = 0; i < 4; i++) {
            talons[i * 2] = (mSwerveMods[i].getTalons()[0]);
            talons[i * 2 + 1] = (mSwerveMods[i].getTalons()[1]);
        }
        return talons;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void enableAutonMoving() {
        autonMovingEnabled = true;
    }
    public void disableAutonMoving() {
        autonMovingEnabled = false;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Command flipHeading(){
        return new InstantCommand(() -> swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), getHeading().rotateBy(Rotation2d.fromDegrees(180)))));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d(); //used to be: return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setOriginalHeading(Rotation2d heading) {
        originalHeading = heading;
    }

    public void updateParallelMotion(boolean parallelModeActive,
                                 boolean returnToOriginal,
                                 boolean allowRotation,
                                 Limelight limelight) {

        // Attempt to get a fresh detected Pose from the limelight
        Pose2d detectedPose = limelight.getAdjustedRobotPose();

        // 1) If we see a new valid pose, update lastKnownTagHeading
        //    (only do this if we actually got a detection!)
        if (detectedPose != null) {
            lastKnownTagHeading = detectedPose.getRotation();
        }

        // 2) If the driver is holding LB (parallelModeActive),
        //    and not holding RB (which would allow rotation),
        //    then forcibly lock heading to the last known tag heading (if we have one).
        if (parallelModeActive && !allowRotation && lastKnownTagHeading != null) {
            setHeading(lastKnownTagHeading);
        }

        // 3) If LB has just been released, revert to the stored original heading
        if (returnToOriginal) {
            setHeading(originalHeading);
        }
    }




    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    

            // SmartDashboard.putNumber("Pigeon ang vel", gyro.getAngularVelocityXDevice().getValueAsDouble());
            SmartDashboard.putNumber("Pigeon Yaw", gyro.getYaw().getValueAsDouble());

                
        }
    }
}       


/*
 * FEIN FEIN FEIN FEIN     FEIN FEIN FEIN FEIN      FEIN FEIN FEIN FEIN FEIN       FEIN FEIN            FEIN
 * FEIN                    FEIN                               FEIN                 FEIN   FEIN          FEIN
 * FEIN                    FEIN                               FEIN                 FEIN      FEIN       FEIN
 * FEIN FEIN FEIN          FEIN FEIN FEIN                     FEIN                 FEIN         FEIN    FEIN
 * FEIN                    FEIN                               FEIN                 FEIN            FEIN FEIN
 * FEIN                    FEIN                               FEIN                 FEIN               FEIN
 * FEIN                    FEIN FEIN FEIN FEIN      FEIN FEIN FEIN FEIN FEIN       FEIN                 FEIN
 */