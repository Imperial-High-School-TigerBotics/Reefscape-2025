package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;

    private CANcoder elevatorCoder;

    private PIDController elevatorMotor1PID;
    private PIDController elevatorMotor2PID;

    DigitalInput limitSwitchTop = new DigitalInput(0);
    DigitalInput limitSwitchBottom = new DigitalInput(1);

    public double ElevatorPos;

    public Elevator(Limelight aprilTagDetection){
        elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor1PID = new PIDController(0,0,0);

        elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.elevatorMotor2ID);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2PID = new PIDController(0,0,0);

        elevatorCoder = new CANcoder(Constants.ElevatorConstants.elevatorCoderID);

        ElevatorPos = Constants.PositionalConstants.allowed_rope_length;
    }

    public TalonFX[] getTalonFXs() {
        TalonFX[] talons = {
            elevatorMotor1,
            elevatorMotor2,
        };
        return talons;
    }
    
    public void clampElevatorSetPos() {
        if (ElevatorPos < Constants.PositionalConstants.min_rope_encoder_value) {
            ElevatorPos = Constants.PositionalConstants.min_rope_encoder_value;
        } else if (ElevatorPos > Constants.PositionalConstants.max_rope_encoder_value) {
            ElevatorPos = Constants.PositionalConstants.max_rope_encoder_value;
        }
    }

    public void setElevator1PID(double position) {
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);

        double speedLimit = 0.3;
        if (setValue > speedLimit) {
            setValue = speedLimit;
        } else if (setValue < -speedLimit) {
            setValue = -speedLimit;
        }

        elevatorMotor1.set(setValue);
    }

    public void LimitSwitchCap() {
    if (limitSwitchTop.get() && limitSwitchBottom.get()) {
        if (limitSwitchTop.get()) {
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
        } else if (limitSwitchBottom.get()) {
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
        }
    } else {
        elevatorMotor1.set(Constants.ElevatorConstants.elevatorMotor1speed);
        elevatorMotor2.set(Constants.ElevatorConstants.elevatorMotor2speed);
        clampElevatorSetPos();
    }

    }

    public void setElevator2PID (double position) {
        double setValue = elevatorMotor2PID.calculate(-getElevatorCoderPos(), position);

        double speedLimit = 0.3;
        if (setValue > speedLimit) {
            setValue = speedLimit;
        } else if (setValue < -speedLimit) {
            setValue = speedLimit;
        }
    }

    public void nextElevatorPID() {
        LimitSwitchCap();
        setElevator1PID(ElevatorPos);
        setElevator2PID(-ElevatorPos);
    }

    public void elevatorUp(double percent) {
        elevatorMotor1.set(percent * Constants.ElevatorConstants.elevatorMotor1speed);
        elevatorMotor2.set(percent * -Constants.ElevatorConstants.elevatorMotor2speed);
    }

    public void elevatorDown(double percent) {
        elevatorMotor1.set(percent * -Constants.ElevatorConstants.elevatorMotor1speed);
        elevatorMotor2.set(percent * Constants.ElevatorConstants.elevatorMotor2speed);
    }

    public void elevatorStop() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
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
