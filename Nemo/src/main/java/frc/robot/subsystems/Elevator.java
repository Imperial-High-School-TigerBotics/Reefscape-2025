package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;

    private CANcoder elevatorCoder;

    private PIDController elevatorMotor1PID;
    private PIDController elevatorMotor2PID;

    private DigitalInput limitSwitchTop = new DigitalInput(Constants.ElevatorConstants.limitSwitchTop);
    private DigitalInput limitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.limitSwitchBottom);

    public double ElevatorPos;

    private ElevatorFeedforward elevatorFeedforward;

    public Elevator() {
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

        elevatorFeedforward = new ElevatorFeedforward(
            Constants.ElevatorConstants.elevatorFeedforwardKs,
            Constants.ElevatorConstants.elevatorFeedforwardKg,
            Constants.ElevatorConstants.elevatorFeedforwardKv,
            Constants.ElevatorConstants.elevatorFeedforwardKa
        );

        elevatorCoder = new CANcoder(Constants.ElevatorConstants.elevatorCoderID);

        ElevatorPos = Constants.ElevatorConstants.min_elevator_pos;
    }

    public void setElevatorPosition(double position) {
        ElevatorPos = position;
        clampElevatorSetPos();
    }

    public void clampElevatorSetPos() {
        ElevatorPos = Math.max(
            Constants.ElevatorConstants.min_elevator_pos, // Ensure minimum position
            Math.min(Constants.ElevatorConstants.max_elevator_pos, ElevatorPos) // Ensure maximum position
        );
    }
    

    public void limitSwitchCap() {
        if (!limitSwitchTop.get()) {
            elevatorStop(); // Stop movement immediately
            ElevatorPos = Constants.ElevatorConstants.max_elevator_pos - Constants.ElevatorConstants.elevatorLimitSwitchOffset;
        } 
        if (!limitSwitchBottom.get()) {
            elevatorStop(); // Stop movement immediately
            ElevatorPos = Constants.ElevatorConstants.min_elevator_pos + Constants.ElevatorConstants.elevatorLimitSwitchOffset;
        }
    }
    

    public void nextElevatorPID() {

        

        clampElevatorSetPos();

        if(ElevatorPos > getElevatorCoderPos() && limitSwitchTop.get() ){

            setElevatorPID(ElevatorPos); // Continue moving if safe


        }else if(ElevatorPos < getElevatorCoderPos() && limitSwitchBottom.get() ){
            setElevatorPID(ElevatorPos);
        }
        else{
            limitSwitchCap();
        }
    }
    
    public void setElevatorCoast(){
        elevatorMotor1.setNeutralMode(NeutralModeValue.Coast);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setElevatorBrake(){
        elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setElevatorPID(double position) {
        double velocity = leftElevatorMotor1RPM();
        double acceleration = (velocity - leftElevatorMotor1RPM()) / Constants.ElevatorConstants.elevatorMotor1speed;

        double feedforward = elevatorFeedforward.calculate(velocity, acceleration);
        double pidOutput = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);

        double setValue = pidOutput + feedforward;

        double speedLimit = Constants.ElevatorConstants.elevatorMotor1speed;

        // if (setValue > speedLimit) {
        //     setValue = speedLimit;
        // } else if (setValue < -speedLimit) {
        //     setValue = -speedLimit;
        // }

        

        elevatorMotor1.setVoltage(-setValue);
        elevatorMotor2.setVoltage(setValue);
    }

    public void setElevatorPID1(double position){
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor1speed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor1.set(-setValue); // Ensure synchronization with motor 2
    }

    public void setElevator2PID(double position) {
        double setValue = elevatorMotor2PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor2speed;
        setValue = Math.max(-speedLimit, Math.min(speedLimit, setValue));

        elevatorMotor2.set(setValue); // Ensure synchronization with motor 1
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

    public void elevatorMove(double speed){
        elevatorMotor1.set(-speed * Constants.ElevatorConstants.elevatorMotor1speed);
        elevatorMotor2.set(speed * Constants.ElevatorConstants.elevatorMotor2speed);
    }

    public void resetElevatorCoder(){
        elevatorCoder.setPosition(0);
    }

    public double getElevMotor1Pos() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }

    public double getElevMotor2Pos() {
        return elevatorMotor2.getPosition().getValueAsDouble();
    }

    public double getElevatorCoderPos() {
        return -elevatorCoder.getPositionSinceBoot().getValueAsDouble();
    }

    public boolean ElevatorAboveHalf() {
        return getElevatorCoderPos() > Constants.ElevatorConstants.max_elevator_pos / 2;
    }

    public double leftElevatorMotor1RPM(){
        return elevatorMotor1.getRotorVelocity().getValueAsDouble();
    }

    public double rightElevatorMotor2RPM(){
        return elevatorMotor2.getRotorVelocity().getValueAsDouble();
    }

    public void elevatorSetVoltage(double voltage){
        elevatorMotor1.setVoltage(-voltage);
        elevatorMotor2.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Pos", getElevatorCoderPos());
        SmartDashboard.putNumber("Target Elevator Pos", ElevatorPos);
        SmartDashboard.putBoolean("Top Limit Switch", !limitSwitchTop.get());
        SmartDashboard.putBoolean("Bottom Limit Switch", !limitSwitchBottom.get());

        SmartDashboard.putNumber("Left Elevator Motor RPM", leftElevatorMotor1RPM());
        SmartDashboard.putNumber("Right Elevator Motor RPM", rightElevatorMotor2RPM());
        nextElevatorPID();
    }

}
