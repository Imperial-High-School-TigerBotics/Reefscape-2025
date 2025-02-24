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

    private DigitalInput limitSwitchTop = new DigitalInput(0);
    private DigitalInput limitSwitchBottom = new DigitalInput(1);

    private double ElevatorPos;

    public Elevator() {
        // Initialize motors with brake mode
        elevatorMotor1 = new TalonFX(Constants.ElevatorConstants.elevatorMotor1ID);
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor1PID = new PIDController(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD
        );

        elevatorMotor2 = new TalonFX(Constants.ElevatorConstants.elevatorMotor2ID);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2PID = new PIDController(
            Constants.ElevatorConstants.elevatorP,
            Constants.ElevatorConstants.elevatorI,
            Constants.ElevatorConstants.elevatorD
        );

        elevatorCoder = new CANcoder(Constants.ElevatorConstants.elevatorCoderID);

        ElevatorPos = Constants.PositionalConstants.allowed_rope_length;
    }

    public void clampElevatorSetPos() {
        ElevatorPos = Math.max(
            Constants.PositionalConstants.min_rope_encoder_value,
            Math.min(Constants.PositionalConstants.max_rope_encoder_value, ElevatorPos)
        );
    }

    public void setElevatorPosition(double position) {
        ElevatorPos = position;
        clampElevatorSetPos();
        nextElevatorPID();
    }

    public void setElevator1PID(double position) {
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = 0.3;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor1.set(setValue);
        elevatorMotor2.set(-setValue); // Manually set motor 2 opposite to motor 1
    }

    public void setElevator2PID(double position) {
        double setValue = elevatorMotor2PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = 0.3;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor2.set(-setValue); // Ensure synchronization with motor 1
    }

    public void limitSwitchCap() {
        if (limitSwitchTop.get()) {
            ElevatorPos = Constants.PositionalConstants.max_rope_encoder_value;
            elevatorStop();
        }
        if (limitSwitchBottom.get()) {
            ElevatorPos = Constants.PositionalConstants.min_rope_encoder_value;
            elevatorStop();
        }
    }

    public void nextElevatorPID() {
        limitSwitchCap();
        if (!limitSwitchTop.get() && !limitSwitchBottom.get()) {
            setElevator1PID(ElevatorPos);
            setElevator2PID(ElevatorPos);
        }
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
