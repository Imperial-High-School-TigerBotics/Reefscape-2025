package frc.robot.autos;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

    /* - Example of how to add auto from Path Planner - 
    private PathPlannerPath path;
    private PathPlannerAuto auto;

    public PathPlannerAuto a1;*/
    public PathPlannerAuto Example_Path;
    


    public Autos(Swerve swerve, Swerve s_Swerve) {
        AutoBuilder.configureHolonomic(
            swerve::getPose, // Robot pose supplier
            swerve::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerve::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
                new PIDConstants(0.2, 0.0, 0.0), // Rotation PID constants
                4.4, // Max module speed, in m/s
                0.431, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            swerve // Reference to this subsystem to set requirements
        );

        // ex: NamedCommands.registerCommand("autoShoot", autoController.autoShoot());

        
    }

    public Command getCurrentAuto() {

        /*if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (a == 1) {
                return a1;
            }
            if (a == 2) {
                return a2;
            }
            return a3;
        }
        if (a == 1) {
            return a3;
        }
        if (a == 2) {
            return a2;
        }
        return a1;//return a3;
        //return AutoBuilder.followPath(path);*/
        return null;
    }
}