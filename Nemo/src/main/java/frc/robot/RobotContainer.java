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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
   /* Controllers */
   private final XboxController driver = new XboxController(0);
   private final XboxController shooter = new XboxController(1);

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


  /* Commands */
  private LimelightCmd limelightCmd;


  private Autos autos;
  private SendableChooser<Command> chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    limelight = new Limelight();
    limelightCmd = new LimelightCmd(limelight);
    limelight.setDefaultCommand(limelightCmd);
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> false,
            driver, limelight
        )
    );

    autos = new Autos(s_Swerve, s_Swerve);
    chooser = new SendableChooser<>();

    configureButtonBindings();
    configureAutoSelector();

  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
}

private void configureAutoSelector() {

  /* - Example from 2024 - 
  chooser.setDefaultOption("shoot & do nothing", autoIntakeAndShooter.autoShoot());

  chooser.addOption("forward 2m", new exampleAuto(s_Swerve));

  chooser.addOption("1note shorter side", autos.a1);
  chooser.addOption("1note middle", autos.a2);
  chooser.addOption("1note longer side", autos.a3);

  chooser.addOption("2note shorter side", autos.two1);
  chooser.addOption("2note middle", autos.two2);
  chooser.addOption("2note longer side", autos.two3);

  chooser.addOption("choose this", autos.g);
  chooser.addOption("3note short side", autos.note3);

  chooser.addOption("2note for use with spyder", autos.notetemp);


  SmartDashboard.putData(chooser);
  */
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }

  public void autonomousPeriodic() {
    
  }
}
