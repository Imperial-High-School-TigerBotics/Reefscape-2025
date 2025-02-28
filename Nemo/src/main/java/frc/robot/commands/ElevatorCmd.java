package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class ElevatorCmd extends Command {
    private final Elevator elevator;
    private final XboxController xbox;

    private double elevatorPos;
    private boolean manualElevatorControl;

    private final SendableChooser<Boolean> manualControlChooser;

    public ElevatorCmd(Elevator elevator, XboxController xbox) {
        this.elevator = elevator;
        addRequirements(this.elevator);

        this.xbox = xbox;
        manualElevatorControl = false;

        elevatorPos = elevator.getElevatorCoderPos();

        // Initialize SendableChooser
        manualControlChooser = new SendableChooser<>();
        manualControlChooser.setDefaultOption("Automatic Elevator Control", false);
        manualControlChooser.addOption("Manual Elevator Control", true);
        SmartDashboard.putData("Elevator Control Mode", manualControlChooser);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
            // Get the selected control mode from the SendableChooser
            manualElevatorControl = manualControlChooser.getSelected();

            boolean bPressed = xbox.getBButton();
            boolean rbPressed = xbox.getRightBumperButton();
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            SmartDashboard.putBoolean("B Button Pressed", bPressed); // Debugging

            if (bPressed) {
                manualElevatorControl = false;
                elevatorPos = Constants.PresetElevatorAndArmConstants.elevatorCoralIntakeFromSourcePos;
                elevator.setElevatorPosition(elevatorPos);
            }

            if (rtPressed) {
                manualElevatorControl = false;
                elevatorPos = Constants.PresetElevatorAndArmConstants.elevatorPickUpAlgaeFromLowerReefPos;
                elevator.setElevatorPosition(elevatorPos);
            }

            if (manualElevatorControl) {
                double axis = -MathUtil.applyDeadband(xbox.getRawAxis(1), Constants.stickDeadband);
                if (axis != 0) {
                    elevatorPos = MathUtil.clamp(
                        elevator.getElevatorCoderPos() + (axis * Constants.ElevatorConstants.axis_multiplier),
                        Constants.ElevatorConstants.min_elevator_pos,
                        Constants.ElevatorConstants.max_elevator_pos
                    );
                    elevator.setElevatorPosition(elevatorPos);
                } else {
                    elevator.setElevatorPosition(elevatorPos); // Maintain last position
                }
            } else if (!bPressed && !rtPressed) {
                // Only move to min position if the elevator is actually above it
                if (elevator.getElevatorCoderPos() > Constants.ElevatorConstants.min_elevator_pos) {
                    elevator.setElevatorPosition(Constants.ElevatorConstants.min_elevator_pos + Constants.ElevatorConstants.elevatorSafety);
                }
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        SmartDashboard.putString("ElevatorCmd", "Ended");
        if (interrupted) {
            SmartDashboard.putString("ElevatorCmd", "Interrupted");
        }
    }
}
