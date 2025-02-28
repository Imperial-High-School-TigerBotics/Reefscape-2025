package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
    private boolean autoTurn;

    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController xbox, Limelight aprilTagDetection) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.limelight = aprilTagDetection;
        this.elevator = elevator;

        this.xbox = xbox;

        autoTurn = false;
    }

    @Override
    public void initialize() {
        autoTurn = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putString("pose", String.valueOf(s_Swerve.swerveOdometry.getPoseMeters().getX()) + ", " + String.valueOf(s_Swerve.swerveOdometry.getPoseMeters().getY()));

        if (xbox.getRightBumperButtonPressed()) {
            autoTurn = !autoTurn;
        }

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = 0;
        if (autoTurn) {
            rotationVal = limelight.getDeadBandedPosition();
        }
        if (!autoTurn || rotationVal == 2) {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }

        double speedLimit = Constants.Swerve.maxSpeed;

        if (elevator.ElevatorAboveHalf()) {
            speedLimit *= Constants.Swerve.ElevatorAboveHalfMultiplier;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedLimit), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}