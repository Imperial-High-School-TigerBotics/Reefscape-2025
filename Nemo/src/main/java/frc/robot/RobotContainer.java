// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.AutoController;
import frc.robot.autos.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmRotationCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.commands.climberCmd;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.ArmRotation;
import frc.robot.commands.ArmShootAndIntakeCmd;
import frc.robot.subsystems.ArmShootAndIntake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private Limelight limelight;
  private Vision vision;
  private Elevator elevator;
  private ArmRotation arm;
  private climber climber;
  private ArmShootAndIntake armShootAndIntake;

  /* Commands */
  private LimelightCmd limelightCmd;


  private Autos autos;
  private AutoController autoController;
  private SendableChooser<Command> chooser;

  private ElevatorCmd elevatorCmd;
  private ArmRotationCmd armCmd;
  private ArmShootAndIntakeCmd armButtonsCmd;

  private climberCmd climberCmd;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    limelight = new Limelight();
    limelightCmd = new LimelightCmd(limelight);
    limelight.setDefaultCommand(limelightCmd);

    elevator = new Elevator();
    elevatorCmd = new ElevatorCmd(elevator, operator);
    elevator.setDefaultCommand(elevatorCmd);

    arm = new ArmRotation();
    armCmd = new ArmRotationCmd(arm, operator);
    arm.setDefaultCommand(armCmd);

    armShootAndIntake = new ArmShootAndIntake();
    armButtonsCmd = new ArmShootAndIntakeCmd(armShootAndIntake, driver);
    armShootAndIntake.setDefaultCommand(armButtonsCmd);

    // climber = new climber();
    // climberCmd = new climberCmd(climber, operator);
    // climber.setDefaultCommand(climberCmd);
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            elevator, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> false,
            driver, limelight
        )
    );

    chooser = new SendableChooser<>();
    autoController = new AutoController(elevator, arm, armShootAndIntake, s_Swerve);
    autos = new Autos(s_Swerve, autoController, s_Swerve);

    configureAutoSelector();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
}

private void configureAutoSelector() {
  chooser.setDefaultOption("Back", new PathPlannerAuto("monkey"));
  chooser.setDefaultOption("Auto Score Coral L2", new PathPlannerAuto("Score L2 Auto"));


  SmartDashboard.putData("Auto Mode", chooser);
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Ensure initial positions are part of a command
    Command initializePositions = new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.ElevatorConstants.elevatorRestPos)),
        new InstantCommand(() -> arm.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos))
    );

    // Get selected auto command, defaulting to a do-nothing command if null
    Command autoCommand = chooser.getSelected();
    if (autoCommand == null) {
        autoCommand = new InstantCommand(); // Default safe command
    }

    // Run initialization, then the selected auto command
    return new SequentialCommandGroup(initializePositions, autoCommand);
}
  

}
