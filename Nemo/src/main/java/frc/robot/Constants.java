// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TestConstants{
    public static final int kTestEncoderChannel = 0;
  }

  public static final double stickDeadband = 0.08;

    public static class TeamDependentFactors {
        public static final boolean redTeam = DriverStation.getAlliance().get() == Alliance.Red;

        public static final int middleAprilTagId = 8;//redTeam ? 4 : 7;
    }

    public static class LimelightConstants {
        public static final String limelightName = "limelight";

        public static final double XOffset = 0.635;
        public static final double YOffset = 0;
        public static final double ZOffset = -0.8382;
    }

    public static class SpeedScaleFactors {
        public static final double autoTurnSpeed = 0.27;
    }

    public static class PositionalConstants {
      /* - Positional Constants for telescoping arm - */
      public static final double rope_encoder_offset = 0.0;
      public static final double min_rope_encoder_value = 0.0;
      public static final double max_rope_encoder_value = 0.0;
      public static final double allowed_rope_length = max_rope_encoder_value - min_rope_encoder_value;

      public static final double arm_encoder_offset = 0.0;
      public static final double arm_clockwise_rotation_encoderCap = 0.0;
      public static final double arm_counterclockwise_rotation_encoderCap = 0.0;
      public static final double arm_rest_encoderPosition = 0.0;
    }


    public static final class Swerve {
      public static final int pigeonID = 49;

      public static final COTSTalonFXSwerveConstants chosenModule = 
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

      /* Drivetrain Constants */
      public static final double trackWidth = Units.inchesToMeters(24.25); // 20.966
      public static final double wheelBase = Units.inchesToMeters(24.25); // 22.8
      public static final double wheelCircumference = chosenModule.wheelCircumference;

      /* Swerve Kinematics 
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
       public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

      /* Module Gear Ratios */
      public static final double driveGearRatio = chosenModule.driveGearRatio;
      public static final double angleGearRatio = chosenModule.angleGearRatio;

      /* Motor Inverts */
      public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
      public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

      /* Angle Encoder Invert */
      public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

      /* Swerve Current Limiting */
      public static final int angleCurrentLimit = 25;
      public static final int angleCurrentThreshold = 40;
      public static final double angleCurrentThresholdTime = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveCurrentLimit = 35;
      public static final int driveCurrentThreshold = 60;
      public static final int driveStatorCurrentLimit = 50;
      public static final double driveCurrentThresholdTime = 0.1;
      public static final boolean driveEnableCurrentLimit = true;


      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
       * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      /* Angle Motor PID Values */
      public static final double angleKP = chosenModule.angleKP;
      public static final double angleKI = chosenModule.angleKI;
      public static final double angleKD = chosenModule.angleKD;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.5; //TODO: modifi later
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.1;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values */
      public static final double driveKS = 2.1; //TODO: modifi later
      public static final double driveKV = 0.0;
      public static final double driveKA = 0.0;

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 3.8; //done
      /** Radians per Second */
      public static final double maxAngularVelocity = 2 * 2 * Math.PI; //done

      /* Neutral Modes */
      public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

      //cancoder offsets
      public static final double offset0 = 75.234375;
      public static final double offset1 = 11.513672;
      public static final double offset2 = 108.193359;
      public static final double offset3 = -58.710938;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class Mod0 { //done
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 2;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset0);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 { //done
          public static final int driveMotorID = 4;
          public static final int angleMotorID = 6;
          public static final int canCoderID = 5;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset1);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      
      /* Back Left Module - Module 2 */
      public static final class Mod2 { //done
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 9;
          public static final int canCoderID = 8;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset2);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 { //done
          public static final int driveMotorID = 10;
          public static final int angleMotorID = 12;
          public static final int canCoderID = 11;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset3);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
      public static final double kMaxSpeedMetersPerSecond = 4.4; //was 3
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
  
      /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

     public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(new PIDConstants(0.1,0.01,0),new PIDConstants(0.2,0,0));
  }

  public static final class ElevatorConstants {
    public static final int elevatorMotor1ID = 66;
    public static final int elevatorMotor2ID = 68;

    public static final int elevatorCoderID = 69;

    public static final double elevatorMotor1speed = 0.5;
    public static final double elevatorMotor2speed = 0.5;
  }

  public static final class ArmConstants {
    public static final int ArmRotator = 0;
    public static final int BallIntake = 1;
    public static final int CoralIntake = 2;

    public static final int ArmEncoder = 1; //TODO: change this

    public static double BallIntakeSpeed = 0.5;
    public static double CoralIntakeSpeed = 0.5;
    public static double ArmRotatorSpeed = 0.5;

    public static boolean ArmRotatorReversed = false;
    public static boolean BallIntakeReversed = false;
    public static boolean CoralIntakeReversed = false;
  }


  public static final RobotConfig CONFIG;
public static RobotConfig config;

  static {
      RobotConfig configTemp;
      try {
          configTemp = RobotConfig.fromGUISettings();
      } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
          configTemp = null; // or provide a default configuration umm
      }
      CONFIG = configTemp;
  }

  //public static final int elevatorMotor = ,

//old code
//* { // Load the RobotConfig from the GUI settings. You should probably
          // store this in your Constants file
         // RobotConfig config;
          //try{
         //   config = RobotConfig.fromGUISettings();
         // } catch (Exception e) {
        //    // Handle exception as needed
       //     e.printStackTrace();
       //  }
  //  }/
}
