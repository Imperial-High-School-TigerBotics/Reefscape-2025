package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private TalonFX ArmRotator;
    private SparkMax BallIntake;
    private SparkMax CoralIntake;

    private PIDController ArmRotatorPID;

    double RotatorPos;

    public Arm(Limelight aprilTagDetection) {
        ArmRotator = new TalonFX(Constants.ArmConstants.ArmRotator);
        ArmRotator.setNeutralMode(NeutralModeValue.Brake);
        ArmRotatorPID = new PIDController(0,0,0);

        BallIntake = new SparkMax(Constants.ArmConstants.BallIntake, MotorType.kBrushless);
        CoralIntake = new SparkMax(Constants.ArmConstants.CoralIntake, MotorType.kBrushless);

    }

    public TalonFX[] getTalonFXs() {
        TalonFX[] talons = {
            ArmRotator,
        };
        return talons;
    }

    public void ArmRotatorCap() {
        if (RotatorPos < Constants.PositionalConstants.arm_counterclockwise_rotation_encoderCap) {
            RotatorPos = Constants.PositionalConstants.arm_counterclockwise_rotation_encoderCap;
        } else if (RotatorPos > Constants.PositionalConstants.arm_clockwise_rotation_encoderCap) {
            RotatorPos = Constants.PositionalConstants.arm_clockwise_rotation_encoderCap;
        }
    }

    public void setArmRotatorPID() {
        double setValue = ArmRotatorPID.calculate(getArmRotatorPos(), RotatorPos);

        double speedLimit = 0.3;
        if (setValue > speedLimit) {
            setValue = speedLimit;
        } else if (setValue < -speedLimit) {
            setValue = -speedLimit;
        }

        ArmRotator.set(setValue);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) > deadzone) {
            return value; // Keep values outside the deadzone
        } else {
            return 0.0; // Set values inside the deadzone to zero
        }
    }

    public void nextArmRotatorPID() {
        ArmRotatorCap();
        setArmRotatorPID();
    }

    public void BallIntakeGo (double speed) {
        BallIntake.set(speed * Constants.ArmConstants.BallIntakeSpeed);
    }

    public void BallIntakeGo(boolean reverse) {
        BallIntake.set((reverse ? -0.5 : 1) * Constants.ArmConstants.BallIntakeSpeed);
    }

    public void CoralIntakeRecieve (double speed) {
        CoralIntake.set(speed * Constants.ArmConstants.CoralIntakeSpeed);
    }

    public void CoralIntakeRecieve(boolean reverse) {
        CoralIntake.set((reverse ? -0.5 : 1) * Constants.ArmConstants.CoralIntakeSpeed);
    }

    public void BallIntakeStop() {
        BallIntake.set(0);
    }

    public void CoralIntakeStop() {
        CoralIntake.set(0);
    }

    public void CoralIntakeIn() {
        CoralIntakeRecieve(false);
    }

    public void CoralIntakeOut() {
        CoralIntakeRecieve(true);
    }

    public void BallIntakeIn() {
        BallIntakeGo(false);
    }

    public void BallIntakeOut() {
        BallIntakeGo(true);
    }

    public void STOPThisMadness() {
        BallIntakeStop();
        CoralIntakeStop();
    }

    public double getArmRotatorPos() {
        return ArmRotator.getPosition().getValueAsDouble();
    }

}