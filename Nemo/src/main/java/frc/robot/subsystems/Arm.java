package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private SparkMax ArmRotator;
    private SparkMax BallIntake;
    private SparkMax CoralIntake;

    private DutyCycleEncoder ArmEncoder;

    public Arm(Limelight aprilTagDetection) {
        ArmRotator = new SparkMax(Constants.ArmConstants.ArmRotator, MotorType.kBrushless);
        BallIntake = new SparkMax(Constants.ArmConstants.BallIntake, MotorType.kBrushless);
        CoralIntake = new SparkMax(Constants.ArmConstants.CoralIntake, MotorType.kBrushless);

        ArmEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoder);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) > deadzone) {
            return value; // Keep values outside the deadzone
        } else {
            return 0.0; // Set values inside the deadzone to zero
        }
    }

    public void BallIntakeGo(boolean reverse) {
        BallIntake.set((reverse ? -0.5 : 1) * Constants.ArmConstants.BallIntakeSpeed);
    }
    public void CoralIntakeRecieve(boolean reverse) {
        CoralIntake.set((reverse ? -0.5 : 1) * Constants.ArmConstants.CoralIntakeSpeed);
    }

}