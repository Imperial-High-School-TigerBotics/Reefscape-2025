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
    private boolean autoShooter;

    public ElevatorCmd(Elevator elevator, XboxController xbox) {
        this.elevator = elevator;
        addRequirements(this.elevator);

        this.xbox = xbox;
        autoShooter = false;

        elevatorPos = elevator.getElevatorCoderPos();
    }

    @Override
    public void initialize() {
        elevator.resetElevatorCoder();
        SmartDashboard.putString("ElevatorCmd", "Initialized");
    }

    @Override 
    public void execute() {
        SmartDashboard.putNumber("Elevator Encoder Pos", elevator.getElevatorCoderPos());

        if (DriverStation.isTeleop()) {
            elevator.setElevatorBrake();
            // Display Elevator Data
            // if(xbox.getRightBumperPressed()){
            //     elevator.resetElevatorCoder();
            // }
            //TODO: Record Elevator Data to constants, then delete this if/else statement and uncomment other code for full implementation
            if(!autoShooter){
                double axis = -MathUtil.applyDeadband(xbox.getRawAxis(1), (Constants.stickDeadband));
                elevator.elevatorMove(axis);
            }else{
                elevator.elevatorStop();
            }
           /* if (!autoShooter) {
                double axis = -MathUtil.applyDeadband(xbox.getRawAxis(1), Constants.stickDeadband);
                if (axis != 0) {
                    elevatorPos += axis;
                    elevator.setElevatorPosition(elevatorPos);
                }else{
                    elevator.elevatorStop();}
            }
        } else {
            elevator.nextElevatorPID();
        */}
    }

    @Override 
    public void end(boolean interrupted) {
        SmartDashboard.putString("ElevatorCmd", "Ended");
        if (interrupted) {
            SmartDashboard.putString("ElevatorCmd", "Interrupted");
        }
        elevator.setElevatorCoast();
    }
}
