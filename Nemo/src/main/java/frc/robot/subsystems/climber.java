package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase{
    public TalonFX climberMotor;

    public climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.climberMotorID);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void climberUp() {
        climberMotor.set(Constants.ClimberConstants.climberMotorspeed);
    }

    public void climberDown() {
        climberMotor.set(-Constants.ClimberConstants.climberMotorspeed);
    }

    public void climberStop() {
        climberMotor.set(0);
    }

}
