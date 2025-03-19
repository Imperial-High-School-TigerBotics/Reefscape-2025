package frc.robot.autos;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Autos {
    /*
     * red2 & blue2 = just aim and shoot
     * 
     */
    private RobotConfig config;


    public Autos(Swerve swerve, AutoController autoController, Swerve s_Swerve) {
        // Load RobotConfig from GUI settings
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return; // Exit constructor if config fails to load
        }

        // Configure AutoBuilder
        AutoBuilder.configure(
            swerve::getPose, // Robot pose supplier
            swerve::setPose, // Reset odometry method
            swerve::getChassisSpeeds, // ChassisSpeeds supplier (must be robot-relative)
            swerve::drive, // Drive method
            new PPHolonomicDriveController( // Built-in holonomic path controller
                new PIDConstants(0.8, 0.0, 0.0), // Translation PID
                new PIDConstants(0.35, 0.0, 0.0)  // Rotation PID
            ),
            config, // Pass the loaded RobotConfig
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
            swerve // Set as the requirement subsystem
        );


        // Register Named Commands and print them
        NamedCommands.registerCommand("Score L2", autoController.scoreCoralL2());

        NamedCommands.registerCommand("Score L4", autoController.scoreCoralL4());

        NamedCommands.registerCommand("Coral Intake From Source", autoController.coralIntakefromSource());

        NamedCommands.registerCommand("resetHeading", new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve));
    }
}