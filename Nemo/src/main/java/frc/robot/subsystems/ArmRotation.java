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

        rotatorPos = 0;
    }

    public void clampArmRotatorSetPos() {
        rotatorPos = Math.max(
            Constants.ArmConstants.ArmMinPos,
            Math.min(Constants.ArmConstants.ArmMaxPos, rotatorPos)
        );
    }

    public void setArmRotatorPosition(double position) {
        rotatorPos = position;
        clampArmRotatorSetPos();
        updateArmPID();
    }

    public void updateArmPID() {
        double setValue = armRotatorPID.calculate(getArmRotatorPos(), rotatorPos);
        double speedLimit = Constants.ArmConstants.ArmRotatorSpeed;
        if (setValue > speedLimit) {
            setValue = speedLimit;
        } else if (setValue < -speedLimit) {
            setValue = -speedLimit;
        }

        armRotator.set(setValue);
    }

    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    public void rotateArmMotor(double speed) {
        armRotator.set(speed * Constants.ArmConstants.ArmRotatorSpeed);
    }

    public double getArmRotatorPos() {
        double normalizedPosition = (armEncoder.get() - Constants.ArmConstants.armCoderOffset) % 1.0;
        if (normalizedPosition < 0) {
            normalizedPosition += 1.0; // Ensure values stay between 0 and 1
        }
        return normalizedPosition;
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Pos", getArmRotatorPos());
        SmartDashboard.putNumber("Target Arm Pos", rotatorPos);
    }
}
