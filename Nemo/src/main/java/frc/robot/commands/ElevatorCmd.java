package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class ElevatorCmd extends Command {
    private final Elevator elevator;
    private final XboxController xbox;

    public ElevatorCmd(Elevator elevator, XboxController xbox) {
        this.elevator = elevator;
        this.xbox = xbox;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        // Get the Y-axis value of the left joystick
        double joystickValue = -MathUtil.applyDeadband(xbox.getLeftY(), Constants.stickDeadband);

        // Scale the joystick value to control the elevator motors
        double motorPercent = joystickValue * Constants.ElevatorConstants.manual_elevator_speed;

        // Set the elevator motor speeds
        elevator.setElevatorPercent(motorPercent);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator motors when the command ends
        elevator.setElevatorPercent(0);
    }
}
