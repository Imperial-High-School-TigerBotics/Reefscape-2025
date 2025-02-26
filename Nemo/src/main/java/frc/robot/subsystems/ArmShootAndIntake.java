package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmShootAndIntake extends SubsystemBase {
    private SparkMax BallIntake;
    private SparkMax CoralIntake;

    public ArmShootAndIntake() {
        BallIntake = new SparkMax(Constants.ArmConstants.BallIntake, MotorType.kBrushless);

        CoralIntake = new SparkMax(Constants.ArmConstants.CoralIntake, MotorType.kBrushless);

    }

    public void updateDashboard(){
        SmartDashboard.putNumber("Ball Intake Speed", BallIntake.get());
        SmartDashboard.putNumber("Coral Intake Speed", CoralIntake.get());
    }

    public void BallIntakeIn() {
        BallIntake.set(Constants.ArmConstants.BallIntakeSpeed);
    }

    public void BallIntakeOut() {
        BallIntake.set(-Constants.ArmConstants.BallIntakeSpeed);
    }

    public void BallIntakeStop() {
        BallIntake.set(0);
    }

    public void CoralIntakeIn() {
        CoralIntake.set(Constants.ArmConstants.CoralIntakeSpeed);
    }

    public void CoralIntakeOut() {
        CoralIntake.set(-Constants.ArmConstants.CoralIntakeSpeed);
    }

    public void CoralIntakeStop() {
        CoralIntake.set(0);
    }

    public void STOPThisMadness() {
        BallIntakeStop();
        CoralIntakeStop();
    }
}
