// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int shooterController = 1;

    public static final double TRIGGER_THRESHOLD = 0.25;
  }


  public static final double stickDeadband = 0.08;

    public static class TeamDependentFactors {
        public static final boolean redTeam = DriverStation.getAlliance().get() == Alliance.Red;
        public static boolean forceRedTeamForTesting = true; // Set true for testing
        public static final double[] reefIDsBlue = {
            18, // closeMiddleReefIDBlue
            19, // closeLeftReefIDBlue
            17, // closeRightReefIDBlue
            21, // farMiddleReefIDBlue
            20, // farLeftReefIDBlue
            22  // farRightReefIDBlue
        };

        public static final double[] reefIDsRed = {
            7,  // closeMiddleReefIDRed
            6,  // closeLeftReefIDRed
            8,  // closeRightReefIDRed
            10, // farMiddleReefIDRed
            11, // farLeftReefIDRed
            9   // farRightReefIDRed
        };

        public static double[] getReefIDs() {
            return redTeam ? reefIDsRed : reefIDsBlue;
        }
    }

    public static class LimelightConstants {
        public static final String limelightName = "limelight";

        public static final double XOffset = 0.635;
        public static final double YOffset = 0;
        public static final double ZOffset = -0.8382;

        //Offset if limelight from set robot heading
        public static final double limelightHeadingOffset = 180;
    }


    public static final class Swerve {
      public static final int pigeonID = 49;

      public static final int SwerveStartHeading = 0;

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
      public static final double ElevatorAboveHalfMultiplier = 0.25;
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
    public static final int elevatorMotor1ID = 21;
    public static final int elevatorMotor2ID = 19;

    public static final int elevatorCoderID = 62;

    /* -----------------Elevator Motor Speeds----------------- */
    public static final double elevatorMotor1speed = 0.5;
    public static final double elevatorMotor2speed = 0.5;
    public static final double manual_elevator_speed  = 1.0;
    /* ------------------------------------------------------ */

    public static final double elevatorP = 0.50;
    public static final double elevatorI = 0.002;
    public static final double elevatorD = 0.0;

    public static final double elevatorSafety = 0.25;
    public static final double elevatorLimitSwitchOffset = 10;


    public static final double min_elevator_pos = 0.0;
    public static final double max_elevator_pos = 7.7;
    public static final double elevatorRestPos = min_elevator_pos + elevatorSafety;

    public static final int limitSwitchTop = 2; 
    public static final int limitSwitchBottom = 1;
  }

  public static final class ArmConstants {
    /* -----------------Arm Motor Speeds----------------- */
    public static final double manual_arm_speed  = 0.3;
    public static double BallIntakeSpeed = 0.3;
    public static double CoralIntakeSpeed = 0.4;
    public static double ArmRotatorSpeed = 0.3;
    /* ------------------------------------------------ */

    public static final int ArmRotator = 14;
    public static final int BallIntake = 18;
    public static final int CoralIntake = 20;

    public static final int ArmEncoder = 0;

    public static double ArmRotatorP = 1;
    public static double ArmRotatorI = 0.0;
    public static double ArmRotatorD = 0.0;
    public static double armCoderOffset = 0.195; // Ball intake left, Coral intake right
    public static double ArmMinPos = 0.19; //Physical Minimum 0.17
    public static double ArmMaxPos = 0.9; //Physical Maximum 0.92
    public static double ArmRestPos = 0.6;
  }

  public static final class PresetElevatorAndArmConstants{

    /*--------------------Coral Intake From Source-------------------*/
    //Mapped to Buttoon B on operator controller
    public static final double armCoralIntakeFromSourcePos = 0.87;
    public static final double elevatorCoralIntakeFromSourcePos = 0.9;
    /*--------------------------------------------------------------*/

    /*--------------------Score Coral L2-------------------*/
    //Mapped to Button Y on operator controller
    public static final double armScoreCoralL2Pos = 0.369;
    public static final double elevatorScoreCoralL2Pos = 2.36;
    /*-----------------------------------------------------*/

    /*--------------------Score Coral L3-------------------*/
    //Mapped to Button X on operator controller
    public static final double armScoreCoralL3Pos = 0.36;
    public static final double elevatorScoreCoralL3Pos = 4.11;
    /*-----------------------------------------------------*/

    /*--------------------Score Coral L4-------------------*/
    //Mapped to Button A on operator controller
    public static final double armScoreCoralL4Pos = 0.37;
    public static final double elevatorScoreCoralL4Pos = 7.65;
    /*-----------------------------------------------------*/

    /*--------------------Score Algae In Processor-------------------*/
    //Mapped to Right Trigger on operator controller
    public static final double armScoreAlgaeInProcessorPos = 0.86;
    public static final double elevatorScoreAlgaeInProcessorPos = 0;
    /*--------------------------------------------------------------*/
    
    /*--------------------Pick Up Algae From Lower Reef In Between L2-L3 -------------------*/
    //Mapped to Right Bumper on operator controller
    public static final double armPickUpAlgaeFromLowerReefPos = 0.74;
    public static final double elevatorPickUpAlgaeFromLowerReefPos = 0.82;
    /*--------------------------------------------------------------------*/

    /*--------------------Pick Up Algae From Upper Reef In Between L3-L4-------------------*/
    //Mapped to Left Bumper on operator controller
    public static final double armPickUpAlgaeFromUpperReefPos = 0.46;
    public static final double elevatorPickUpAlgaeFromUpperReefPos = 4.71;
    /*--------------------------------------------------------------------*/

    /*----------------------Score Into Barge--------------------*/
    //Mapped to Left Trigger on operator controller
    public static final double armScoreIntoBargePos = 0.42;
    public static final double elevatorScoreIntoBargePos = 7.55;
    /*----------------------------------------------------------*/
  }

  public static final class ClimberConstants {
    public static final int climberMotorID = 45;

    public static final double climberMotorspeed = 0.2;

    public static final double climberRestPos = 0.0;
    public static final double climberMinPos = 0.0;
    public static final double climberMaxPos = 0.0;

    public static final double climberP = 1;
    public static final double climberI = 0.0;
    public static final double climberD = 0.0;
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
