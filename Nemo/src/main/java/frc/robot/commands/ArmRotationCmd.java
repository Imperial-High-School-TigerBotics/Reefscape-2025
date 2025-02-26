package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRoatation;

public class ArmRotationCmd extends Command {
    private final ArmRoatation arm;
    private final XboxController xbox;

    private boolean autoShooter;
    private double rotatorPos;

    public ArmRotationCmd(ArmRoatation arm, XboxController xbox) {
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

            // //TODO: Record Arm Data to constants, then delete this if/else statement and uncomment other code for full implementation

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
        //arm.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
    }
}
