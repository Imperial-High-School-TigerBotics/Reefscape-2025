package frc.robot.autos;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

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
    

    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(new PIDConstants(0.1,0,0),new PIDConstants(0.2,0,0));

 /* public Autos(Swerve swerve, AutoController autoController, Swerve s_Swerve, Vision vision) {
    
    AutoBuilder.configure(
      vision::getPoseEstimation,
      swerve::setPose,
      swerve::getChassisSpeeds,
      swerve::drive, 
      kDriveController, 
    

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
      this // Reference to this subsystem to set requirements
);
          } */


        // ex: NamedCommands.registerCommand("autoShoot", autoController.autoShoot());

        
    

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