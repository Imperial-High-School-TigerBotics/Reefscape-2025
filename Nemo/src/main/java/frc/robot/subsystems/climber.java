package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase{
    public TalonFX climberMotor;

    public PIDController  climberPID;

    double climberPos;

    public climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.climberMotorID);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
        climberPID = new PIDController(
            Constants.ClimberConstants.climberP,
            Constants.ClimberConstants.climberI,
            Constants.ClimberConstants.climberD
        );

        climberPos = Constants.ClimberConstants.climberRestPos;
    }

    public void clampElevatorSetPos() {
        climberPos = Math.max(
            Constants.ClimberConstants.climberMinPos,
            Math.min(Constants.ClimberConstants.climberMaxPos, climberPos)
        );
    }

    public double getClimberPos() {
        return climberMotor.getPosition().getValueAsDouble();
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
