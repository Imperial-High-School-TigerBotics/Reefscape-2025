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
    private BooleanSupplier allowRotationSup;
    private boolean wasParallelModeActive = false;
    private double storedHeading = 0;
    private double lastKnownTagYaw = 0;

    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController xbox, Limelight aprilTagDetection, BooleanSupplier parallelMotionsup, BooleanSupplier allowRotationSup) {
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
        String.valueOf(s_Swerve.swerveOdometry.getPoseMeters().getX()) + ", " 
        + String.valueOf(s_Swerve.swerveOdometry.getPoseMeters().getY()));


    // NEW: detect the "just pressed" event
    boolean justPressedLB = (parallelMotionSup.getAsBoolean() && !wasParallelModeActive);
    // If we release LB, we want to reset heading
    boolean returnToOriginal = !parallelMotionSup.getAsBoolean() && wasParallelModeActive;

    // NEW: If we just pressed LB for the first time, store our current heading
    if (justPressedLB) {
        s_Swerve.setOriginalHeading(s_Swerve.getHeading());
    }

    s_Swerve.updateParallelMotion(parallelMotionSup.getAsBoolean(), returnToOriginal, allowRotationSup.getAsBoolean(), limelight);

    wasParallelModeActive = parallelMotionSup.getAsBoolean(); // Keep track if parallel mode was active

    // --- normal driving logic below ---
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal      = MathUtil.applyDeadband(strafeSup.getAsDouble(),      Constants.stickDeadband);
    double rotationVal    = MathUtil.applyDeadband(rotationSup.getAsDouble(),    Constants.stickDeadband);

    // If LB is pressed but RB is not, lock rotation
    if (parallelMotionSup.getAsBoolean() && !allowRotationSup.getAsBoolean()) {
        rotationVal = 0;
    }

    double speedLimit = Constants.Swerve.maxSpeed;
    if (elevator.ElevatorAboveHalf()) {
        speedLimit *= Constants.Swerve.ElevatorAboveHalfMultiplier;
    }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(speedLimit), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        !robotCentricSup.getAsBoolean(), 
        true
    );
}



}