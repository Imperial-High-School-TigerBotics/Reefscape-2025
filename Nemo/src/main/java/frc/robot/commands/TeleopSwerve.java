package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TeamDependentFactors;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final Elevator elevator;
    private final Limelight limelight;

    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;

    // Example: “Hold this to aim to a nearest AprilTag”
    private final BooleanSupplier aimToTagButton;

    // For motion-limiting while elevator is up, etc.
    private static final double ELEVATOR_LIMIT_FACTOR = Constants.Swerve.ElevatorAboveHalfMultiplier;  

    public TeleopSwerve(
            Swerve s_Swerve,
            Elevator elevator,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            XboxController driver,
            Limelight limelight,
            BooleanSupplier aimToTagButton) {

        this.s_Swerve = s_Swerve;
        this.elevator = elevator;
        this.limelight = limelight;
        addRequirements(s_Swerve); // drive subsystem

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.aimToTagButton = aimToTagButton;
    }

    @Override
    public void execute() {
        // 1) Normal drive inputs
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // 2) Optionally reduce speeds if elevator is above half
        double speedLimit = Constants.Swerve.maxSpeed;
        if(elevator.ElevatorAboveHalf()){
            speedLimit *= ELEVATOR_LIMIT_FACTOR;
        }

        // 3) If the “Aim to Tag” button is held, override rotationVal to face the nearest tag
        if(aimToTagButton.getAsBoolean()){
            // Get the nearest valid tag ID for your alliance:
            double[] validTagIds = TeamDependentFactors.getReefIDs(); // or whichever you prefer
            double closestTagId = limelight.getClosestTag(validTagIds);

            if(closestTagId >= 0) {
                // If we found a valid tag, do a small “point at the tag’s position on the field.”
                // We'll estimate the tag’s location from the known april-tag field layout 
                // or from the “limelight.getTarget(...)”. For a quick example, let’s do:
                double[] tagData = limelight.getTarget((int)closestTagId);
                // Suppose tagData is {id, fieldX, fieldY} – you might store actual known positions, etc.

                if(tagData != null && tagData.length >= 3) {
                    // Robot’s current pose
                    Pose2d robotPose = s_Swerve.getPose();

                    double tagX = tagData[1];  // Possibly you store real field coords
                    double tagY = tagData[2];

                    // Vector from robot to the tag
                    double dx = tagX - robotPose.getX();
                    double dy = tagY - robotPose.getY();
                    
                    // Desired heading = atan2(dy, dx)
                    double desiredHeading = Math.atan2(dy, dx);

                    // Current heading (in radians)
                    double currentHeading = robotPose.getRotation().getRadians();

                    double headingError = desiredHeading - currentHeading;
                    // Normalize to -pi..pi
                    headingError = MathUtil.angleModulus(headingError);

                    // Some small P-gain for turning
                    double kP = 1.5;
                    rotationVal = kP * headingError;

                    // Optionally clamp max rotation
                    rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);
                }
            }
        }

        // 4) Drive
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
