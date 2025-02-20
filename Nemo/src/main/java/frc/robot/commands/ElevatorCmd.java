package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class ElevatorCmd extends Command{
    private Elevator elevator;

    private XboxController xbox;

    private double ElevatorPos;
    private boolean autoShooter;

    public ElevatorCmd (Elevator elevator, XboxController xbox) {
        this.elevator = elevator;
        addRequirements(this.elevator);

        this.xbox = xbox;
        autoShooter = false;

        ElevatorPos = elevator.getElevatorCoderPos();
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()){

            SmartDashboard.putNumber("Elevator Motor 1 pos", elevator.getElevMotor1Pos());
            SmartDashboard.putNumber("Elevator Motor 2 Pos", elevator.getElevMotor2Pos());
            SmartDashboard.putNumber("Elevator Coder Pos", elevator.getElevatorCoderPos());

            if (!autoShooter) {
                double axis = xbox.getRawAxis(0);
                elevator.ElevatorPos += MathUtil.applyDeadband(axis, .1) * 0.1;
                elevator.nextElevatorPID();
            }
        } else {
            elevator.nextElevatorPID();
        }
    }
}
