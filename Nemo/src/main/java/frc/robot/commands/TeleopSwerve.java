package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.TeamDependentFactors;
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
    private BooleanSupplier allowRotationSup;
    private boolean wasParallelModeActive = false;
    private double storedHeading = 0;
    private double lastKnownTagYaw = 0;

    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController xbox, Limelight aprilTagDetection, BooleanSupplier parallelMotionSup, BooleanSupplier allowRotationSup) {
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
        this.allowRotationSup = allowRotationSup;


    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putString("pose", 
            s_Swerve.getPose().getX() + ", " + s_Swerve.getPose().getY());

        // Track button states
        boolean parallelMotionActive = parallelMotionSup.getAsBoolean(); // A button
        boolean allowRotation = allowRotationSup.getAsBoolean(); // B button
        boolean justPressedA = parallelMotionActive && !wasParallelModeActive;
        boolean justReleasedA = !parallelMotionActive && wasParallelModeActive;

        // If A is just pressed, find closest AprilTag and store its heading
        if (justPressedA) {
            double[] validTagIds = TeamDependentFactors.getReefIDs();
            double closestTagId = limelight.getClosestTag(validTagIds);

            if (closestTagId != -1) {
                double[] tagData = limelight.getTarget((int) closestTagId);
                if (tagData != null) {
                    lastKnownTagYaw = tagData[0]; // Extract tag yaw (adjust if needed) 
                }
            }

            // Store the original heading to return to when A is released
            storedHeading = s_Swerve.getHeading().getDegrees();
        }

        // If A is held, force the heading to the last known AprilTag yaw
        if (parallelMotionActive && !allowRotation) {
            s_Swerve.setHeading(Rotation2d.fromDegrees(lastKnownTagYaw - storedHeading)); //adjust if necessary
            
        }

        // If A is released, return to the stored heading
        if (justReleasedA) {
            s_Swerve.setHeading(Rotation2d.fromDegrees(storedHeading));
        }

        // Normal swerve drive behavior
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // If A is held and B is NOT held, lock rotation
        if (parallelMotionActive && !allowRotation) {
            rotationVal = 0;
        }

        // Apply speed limits based on elevator height
        double speedLimit = Constants.Swerve.maxSpeed;
        if (elevator.ElevatorAboveHalf()) {
            speedLimit *= Constants.Swerve.ElevatorAboveHalfMultiplier;
        }

        // Drive the swerve
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        // Track previous state for edge detection
        wasParallelModeActive = parallelMotionActive;
    }





}