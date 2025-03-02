package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private Limelight limelight;
    private Elevator elevator;

    private XboxController xbox;
    private BooleanSupplier parallelMotionSup;
    private boolean wasParallelModeActive = false;
    private double storedHeading = 0;
    private double lastKnownTagYaw = 0;

    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController xbox, Limelight aprilTagDetection, BooleanSupplier parallelMotionsup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.limelight = aprilTagDetection;
        this.elevator = elevator;

        this.xbox = xbox;

        this.parallelMotionSup = parallelMotionSup;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean parallelModeActive = parallelMotionSup.getAsBoolean();

        if (parallelModeActive && !wasParallelModeActive) {
            // Capture the robot's current heading
            storedHeading = s_Swerve.getHeading().getDegrees();

            // Get the closest valid AprilTag for reef alignment
            double closestTagId = Constants.TeamDependentFactors.redTeam ?
                                  limelight.getClosestTag(new double[]{7, 6, 8, 10, 11, 9}) :  // Red Team IDs
                                  limelight.getClosestTag(new double[]{18, 19, 17, 21, 20, 22}); // Blue Team IDs

            double[] tagData = limelight.getTarget((int) closestTagId);
            if (tagData != null) {
                lastKnownTagYaw = tagData[2]; // Store last detected yaw of the tag
            }
        }

        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (parallelModeActive) {
            // Maintain heading at last known AprilTag direction
            s_Swerve.setHeading(Rotation2d.fromDegrees(lastKnownTagYaw));
            rotationVal = 0; // Prevents manual rotation

            // Drive parallel while keeping heading fixed
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                0, // No rotation
                !robotCentricSup.getAsBoolean(),
                true
            );
        } else {
            // Restore the original heading when the button is released
            if (wasParallelModeActive) {
                s_Swerve.setHeading(Rotation2d.fromDegrees(storedHeading));
            }

            // Normal driving behavior
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
            );
        }

        wasParallelModeActive = parallelModeActive;
    }


}