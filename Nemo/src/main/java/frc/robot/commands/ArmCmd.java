package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmCmd extends Command {
    private final Arm arm;
    private final XboxController xbox;

    private boolean autoShooter;
    private double rotatorPos;

    public ArmCmd(Arm arm, XboxController xbox) {
        this.arm = arm;
        this.xbox = xbox;
        addRequirements(arm);

        autoShooter = false;
        rotatorPos = arm.getArmRotatorPos();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
            // Ball Intake Controls
            if (xbox.getYButtonPressed()) {
                arm.BallIntakeIn();
            } 
            if (xbox.getYButtonReleased()) {
                arm.BallIntakeStop();
            }

            if (xbox.getBButtonPressed()) {
                arm.BallIntakeOut();
            } 
            if (xbox.getBButtonReleased()) {
                arm.BallIntakeStop();
            }

            // Coral Intake Controls
            if (xbox.getXButtonPressed()) {
                arm.CoralIntakeIn();
            } 
            if (xbox.getXButtonReleased()) {
                arm.CoralIntakeStop();
            }

            if (xbox.getAButtonPressed()) {
                arm.CoralIntakeOut();
            } 
            if (xbox.getAButtonReleased()) {
                arm.CoralIntakeStop();
            }

            //TODO: Record Arm Data to constants, then delete this if/else statement and uncomment other code for full implementation

            if(!autoShooter){
                double axis = xbox.getRawAxis(4);
                axis = MathUtil.applyDeadband(axis, Constants.stickDeadband);

                arm.rotateArmMotor(axis);
            }else{
                arm.rotateArmMotor(0);
            }
            /*
            // Manual Arm Control
            if (!autoShooter) {
                double axis = xbox.getRawAxis(4); // Inverted for correct movement
                axis = MathUtil.applyDeadband(axis, 0.1); // Apply deadband to remove minor input noise
                
                // Update the arm position based on joystick input
                rotatorPos += axis * Constants.ArmConstants.ArmRotatorSpeed;
                arm.setArmRotatorPosition(rotatorPos);
         }else {
            arm.setArmRotatorPosition(rotatorPos); // Maintain last position in non-teleop modes
        */}

        // Send data to SmartDashboard for debugging
        SmartDashboard.putNumber("Arm Encoder Pos", arm.getArmRotatorPos());
        SmartDashboard.putNumber("Target Arm Pos", rotatorPos);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
    }
}
