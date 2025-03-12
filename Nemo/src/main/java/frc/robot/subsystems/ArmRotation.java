package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotation extends SubsystemBase {
    private final TalonFX armRotator;
    private final PIDController armRotatorPID;
    private final DutyCycleEncoder armEncoder;

    private double rotatorPos;

    public ArmRotation() {
        armRotator = new TalonFX(Constants.ArmConstants.ArmRotator);
        armRotator.setNeutralMode(NeutralModeValue.Brake);

        armRotatorPID = new PIDController(
            Constants.ArmConstants.ArmRotatorP,
            Constants.ArmConstants.ArmRotatorI,
            Constants.ArmConstants.ArmRotatorD
        );

        armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoder);

        rotatorPos = Constants.ArmConstants.ArmMinPos;
    }

    public void setArmRotatorPosition(double position) {
        rotatorPos = position;
        clampArmRotatorSetPos();
    }

    public void clampArmRotatorSetPos() {
        rotatorPos = Math.max(
            Constants.ArmConstants.ArmMinPos,
            Math.min(Constants.ArmConstants.ArmMaxPos, rotatorPos)
        );
    }

    public void nextArmPID() {
        clampArmRotatorSetPos();
        setArmRotatorPID(rotatorPos);
    }

    public void setArmRotatorPID(double position) {
        double setValue = armRotatorPID.calculate(getArmRotatorPos(), position);
        double speedLimit = Constants.ArmConstants.ArmRotatorSpeed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));
        armRotator.set(setValue);
    }

    public void rotateArmMotor(double speed) {
        armRotator.set(speed * Constants.ArmConstants.ArmRotatorSpeed);
    }

    public double getArmRotatorPos() {
        double normalizedPosition = (armEncoder.get()) % 1.0;
        if (normalizedPosition < 0) {
            normalizedPosition += 1.0;
        }
        return normalizedPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Pos", getArmRotatorPos());
        SmartDashboard.putNumber("Target Arm Pos", rotatorPos);
        nextArmPID();
    }
}
