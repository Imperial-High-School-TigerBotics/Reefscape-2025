package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
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

    public ElevatorCmd(Elevator elevator, XboxController xbox) {
        this.elevator = elevator;
        addRequirements(this.elevator);

        this.xbox = xbox;
        manualElevatorControl = false;

        elevatorPos = elevator.getElevatorCoderPos();
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
           if (!manualElevatorControl) {
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
