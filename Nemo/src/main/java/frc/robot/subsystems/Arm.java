package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private TalonFX ArmRotator;
    private SparkMax BallIntake;
    private SparkMax CoralIntake;

    private PIDController ArmRotatorPID;
    private DutyCycleEncoder ArmEncoder; 

    public double RotatorPos;

    public Arm(Limelight aprilTagDetection) {
        ArmRotator = new TalonFX(Constants.ArmConstants.ArmRotator);
        ArmRotator.setNeutralMode(NeutralModeValue.Brake);
        ArmRotatorPID = new PIDController(
            Constants.ArmConstants.ArmRotatorP,
            Constants.ArmConstants.ArmRotatorI,
            Constants.ArmConstants.ArmRotatorD
        );

        ArmEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoder);

        BallIntake = new SparkMax(Constants.ArmConstants.BallIntake, MotorType.kBrushless);
        CoralIntake = new SparkMax(Constants.ArmConstants.CoralIntake, MotorType.kBrushless);

        RotatorPos = Constants.ArmConstants.ArmRestPos;

    }

    public TalonFX[] getTalonFXs() {
        TalonFX[] talons = {
            ArmRotator,
        };
        return talons;
    }


    public void clampArmRotatorSetPos(){
        RotatorPos = Math.max(
            Constants.ArmConstants.ArmMinPos,
            Math.min(Constants.ArmConstants.ArmMaxPos, RotatorPos)
        );
    }

    public void setArmRotatorPosition(double position) {
        RotatorPos = position;
        clampArmRotatorSetPos();
        nextArmRotatorPID();
    }

    public void nextArmRotatorPID() {
        double setValue = ArmRotatorPID.calculate(getArmRotatorPos(), RotatorPos);

        double speedLimit = 0.3;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        ArmRotator.set(setValue);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) > deadzone) {
            return value; // Keep values outside the deadzone
        } else {
            return 0.0; // Set values inside the deadzone to zero
        }
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

    public void rotateArmMotor(double speed) {
        ArmRotator.set(speed * Constants.ArmConstants.ArmRotatorSpeed);
    }

    public double getArmRotatorPos() {
        return ArmRotator.getPosition().getValueAsDouble();
    }

}