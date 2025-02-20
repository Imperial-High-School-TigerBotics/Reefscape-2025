package frc.robot.autos;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Autos {
    // This class contains autonomous routines for the robot.
    // (Other auto-related code remains unchanged)
    
    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
            new PIDConstants(0.1, 0, 0), new PIDConstants(0.2, 0, 0));

    public Autos(frc.robot.subsystems.Swerve swerve, AutoController autoController, 
                 frc.robot.subsystems.Swerve s_Swerve, frc.robot.subsystems.Vision vision) {
        // Autonomous routines initialization (if needed)
    }
}

class DriveSubsystem extends SubsystemBase {
    // Create a Pigeon2 from the CTR library. Adjust the CAN ID (here 0 is used as an example).
    private final Pigeon2 pigeon = new Pigeon2(0);
    // Keep track of the robot pose. This now includes position and heading.
    private Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    // Variables for integrating acceleration to obtain velocity and position.
    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double positionX = 0.0;
    private double positionY = 0.0;
    // Assume a periodic update rate of 20ms.
    private static final double dt = 0.02;

//where old auto builder was
//
//
//
// 
    // Called periodically; update both heading and position using Pigeon2 acceleration.
    @Override
    public void periodic() {
        // Retrieve yaw in degrees and convert to radians.
        double yawDegrees = ((Rotation2d) pigeon.getYaw().getValue()).getDegrees();
        double yawRadians = Math.toRadians(yawDegrees);

        // Retrieve the acceleration values; since the current Pigeon2 does not support getAccelerometer, set acceleration to 0.0.
        double accelX = 0.0;
        double accelY = 0.0;

        // Rotate the robot-relative acceleration into the field coordinate frame.
        Translation2d robotAccel = new Translation2d(accelX, accelY);
        Translation2d fieldAccel = robotAccel.rotateBy(new Rotation2d(yawRadians));

        // Integrate acceleration to update velocity.
        velocityX += fieldAccel.getX() * dt;
        velocityY += fieldAccel.getY() * dt;

        // Integrate velocity to update position.
        positionX += velocityX * dt;
        positionY += velocityY * dt;

        // Update the pose with the new position and current heading.
        pose = new Pose2d(positionX, positionY, new Rotation2d(yawRadians));
    }

    // Return the current pose (position and heading)

}
