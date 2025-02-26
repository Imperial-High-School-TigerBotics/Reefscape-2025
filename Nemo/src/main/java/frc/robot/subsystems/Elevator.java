package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;

    private CANcoder elevatorCoder;

    private PIDController elevatorMotor1PID;
    private PIDController elevatorMotor2PID;

    private DigitalInput limitSwitchTop = new DigitalInput(Constants.ElevatorConstants.limitSwitchTop);
    private DigitalInput limitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.limitSwitchBottom);

    private double ElevatorPos;

    public Elevator() {
        // Initialize motors with brake mode
        elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Coast);
        elevatorMotor1PID = new PIDController(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD
        );

        elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.elevatorMotor2ID);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
        elevatorMotor2PID = new PIDController(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD
        );

        elevatorCoder = new CANcoder(Constants.ElevatorConstants.elevatorCoderID);

        ElevatorPos = Constants.ElevatorConstants.min_elevator_pos;
    }

    public void setElevatorPosition(double position) {
        ElevatorPos = position;
        clampElevatorSetPos();
        nextElevatorPID();
    }

    public void clampElevatorSetPos() {
        ElevatorPos = Math.max(
            Constants.ElevatorConstants.min_elevator_pos,
            Math.min(Constants.ElevatorConstants.max_elevator_pos, ElevatorPos)
        );
    }

    public void limitSwitchCap() {
        if (limitSwitchTop.get()) {
            elevatorStop(); // Stop movement immediately
            ElevatorPos = Constants.ElevatorConstants.max_elevator_pos - Constants.ElevatorConstants.elevatorLimitSwitchOffset;
        } 
        if (limitSwitchBottom.get()) {
            elevatorStop(); // Stop movement immediately
            ElevatorPos = Constants.ElevatorConstants.min_elevator_pos + Constants.ElevatorConstants.elevatorLimitSwitchOffset;
        }
    }
    

    public void nextElevatorPID() {
        limitSwitchCap(); // Check limit switches and adjust ElevatorPos if necessary
    
        // Prevent movement in the direction of the limit switch
        if (limitSwitchTop.get() && ElevatorPos >= Constants.ElevatorConstants.max_elevator_pos) {
            return; // Do nothing if we're at the top and trying to move up
        }
        if (limitSwitchBottom.get() && ElevatorPos <= Constants.ElevatorConstants.min_elevator_pos) {
            return; // Do nothing if we're at the bottom and trying to move down
        }
    
        setElevatorPID(ElevatorPos); // Continue moving if safe
    }
    
    public void setElevatorCoast(){
        elevatorMotor1.setNeutralMode(NeutralModeValue.Coast);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setElevatorBrake(){
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setElevatorPID(double position) {
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor1speed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor1.set(-setValue);
        elevatorMotor2.set(setValue);
    }

    public void setElevatorPID1(double position){
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor1speed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor1.set(-setValue); // Ensure synchronization with motor 2
    }

    public void setElevator2PID(double position) {
        double setValue = elevatorMotor2PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor2speed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor2.set(setValue); // Ensure synchronization with motor 1
    }


    public void elevatorUp(double percent) {
        double speed = percent * Constants.ElevatorConstants.elevatorMotor1speed;
        elevatorMotor1.set(speed);
        elevatorMotor2.set(-speed); // Opposite direction for synchronization
    }

    public void elevatorDown(double percent) {
        double speed = percent * -Constants.ElevatorConstants.elevatorMotor1speed;
        elevatorMotor1.set(speed);
        elevatorMotor2.set(-speed); // Opposite direction for synchronization
    }

    public void elevatorStop() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }

    public void elevatorMove(double speed){
        elevatorMotor1.set(-speed * Constants.ElevatorConstants.elevatorMotor1speed);
        elevatorMotor2.set(speed * Constants.ElevatorConstants.elevatorMotor2speed);
    }

    public void resetElevatorCoder(){
        elevatorCoder.setPosition(0);
    }

    public double getElevMotor1Pos() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    public double getElevMotor2Pos() {
        return elevatorMotor2.getPosition().getValueAsDouble();
    }

    public double getElevatorCoderPos() {
        return elevatorCoder.getPosition().getValueAsDouble();
    }
}
