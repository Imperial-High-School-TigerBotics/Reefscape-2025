package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private DigitalInput limitSwitchTop = new DigitalInput(Constants.ElevatorConstants.limitSwitchTop);
    private DigitalInput limitSwitchBottom = new DigitalInput(Constants.ElevatorConstants.limitSwitchBottom);

    public double ElevatorPos;

    public double ElevatorP;
    public double ElevatorI;
    public double ElevatorD;

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

            ElevatorPos = Constants.ElevatorConstants.min_elevator_pos;
        }
   //TODO #2 for alex check if until line 98 is correct
   //Does this allow for us to click a button and have the pid tuning change? and then is this the correct method to do this?
    public void adjustPIDValues() {
        boolean increaseP = SmartDashboard.getBoolean("Increase P", false);
        boolean decreaseP = SmartDashboard.getBoolean("Decrease P", false);
        boolean increaseI = SmartDashboard.getBoolean("Increase I", false);
        boolean decreaseI = SmartDashboard.getBoolean("Decrease I", false);
        boolean increaseD = SmartDashboard.getBoolean("Increase D", false);
        boolean decreaseD = SmartDashboard.getBoolean("Decrease D", false);
        SmartDashboard.putBoolean("Increase P", false);
        SmartDashboard.putBoolean("Decrease P", false);
        SmartDashboard.putBoolean("Increase I", false);
        SmartDashboard.putBoolean("Decrease I", false);
        SmartDashboard.putBoolean("Increase D", false);
        SmartDashboard.putBoolean("Decrease D", false);

        if (increaseP) {
            ElevatorP += 0.05;
        } else if (decreaseP) {
            ElevatorP -= 0.05;
        }

        if (increaseI) {
            ElevatorI += 0.005;
        } else if (decreaseI) {
            ElevatorI -= 0.005;
        }

        if (increaseD) {
            ElevatorD += 0.01;
        } else if (decreaseD) {
            ElevatorD -= 0.01;
        }

        elevatorMotor1PID.setPID(ElevatorP, ElevatorI, ElevatorD);
        elevatorMotor2PID.setPID(ElevatorP, ElevatorI, ElevatorD);

        SmartDashboard.putNumber("Elevator P", ElevatorP);
        SmartDashboard.putNumber("Elevator I", ElevatorI);
    SmartDashboard.putNumber("Elevator D", ElevatorD);
        SmartDashboard.putNumber("Elevator D", ElevatorD);
    {
        System.out.println("Elevator P: " + ElevatorP);
        System.out.println("Elevator I: " + ElevatorI);
        System.out.println("Elevator D: " + ElevatorD);
        }

        elevatorCoder = new CANcoder(Constants.ElevatorConstants.elevatorCoderID);

        ElevatorPos = Constants.ElevatorConstants.min_elevator_pos; }
    

    public void setElevatorPosition(double position) {
        ElevatorPos = position;
        clampElevatorSetPos();
       // nextElevatorPID();
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
           // elevatorStop();
        }

        // if ((!limitSwitchTop.get() && ElevatorPos >= Constants.ElevatorConstants.max_elevator_pos) ||
        // (!limitSwitchBottom.get() && ElevatorPos <= Constants.ElevatorConstants.min_elevator_pos)) 
        // {
        //     return; // Stop movement if at limits
        // // }
        // if(ElevatorPos > getElevatorCoderPos() && limitSwitchTop.get()){

        //     setElevatorPID(ElevatorPos); // Continue moving if safe


        // }
        // else if(ElevatorPos < getElevatorCoderPos() && limitSwitchBottom.get()){

        //     setElevatorPID(ElevatorPos); // Continue moving if safe


        // }
        // setElevatorPID(ElevatorPos); // Continue moving if safe
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
        double setValue = elevatorMotor1PID.calculate(getElevatorCoderPos(), position);
        double speedLimit = Constants.ElevatorConstants.elevatorMotor1speed;
        if (setValue > speedLimit) {
            setValue = speedLimit;
        } else if (setValue < -speedLimit) {
            setValue = -speedLimit;
        }

        

        elevatorMotor1.set(-setValue);
        elevatorMotor2.set(setValue);
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
        return elevatorCoder.getPositionSinceBoot().getValueAsDouble();
    }

    public boolean ElevatorAboveHalf() {
        return getElevatorCoderPos() > Constants.ElevatorConstants.max_elevator_pos / 2;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Pos", getElevatorCoderPos());
        SmartDashboard.putBoolean("Top Limit Switch", !limitSwitchTop.get());
        SmartDashboard.putBoolean("Bottom Limit Switch", !limitSwitchBottom.get());

        // Read values from SmartDashboard
        ElevatorP = SmartDashboard.getNumber("Elevator P", Constants.ElevatorConstants.elevatorP);
        ElevatorI = SmartDashboard.getNumber("Elevator I", Constants.ElevatorConstants.elevatorI);
        ElevatorD = SmartDashboard.getNumber("Elevator D", Constants.ElevatorConstants.elevatorD);

        elevatorMotor1PID.setPID(ElevatorP, ElevatorI, ElevatorD);
        elevatorMotor2PID.setPID(ElevatorP, ElevatorI, ElevatorD);

        nextElevatorPID();
    }


}
