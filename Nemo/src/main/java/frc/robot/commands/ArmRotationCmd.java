package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotation;

public class ArmRotationCmd extends Command {
    private final ArmRotation arm;
    private final XboxController xbox;

    private boolean autoShooter;
    private double rotatorPos;

    public ArmRotationCmd(ArmRotation arm, XboxController xbox) {
        this.arm = arm;
        this.xbox = xbox;
        addRequirements(arm);

        autoShooter = false;
        rotatorPos = arm.getArmRotatorPos();
    }

    @Override
    public void execute() {
        // if (DriverStation.isTeleop()) {
        //     if (!autoShooter) {
        //         double axis = MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
        //         rotatorPos += axis * Constants.ArmConstants.ArmRotatorSpeed;
        //         arm.setArmRotatorPosition(rotatorPos);
        //     }
        // }

        if(DriverStation.isTeleop()){
            if(!autoShooter){
                double axis = MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
                arm.rotateArmMotor(axis * Constants.ArmConstants.ArmRotatorSpeed);
            }
        }

        // Displaying encoder values for debugging
        SmartDashboard.putNumber("Arm Encoder Pos", arm.getArmRotatorPos());
        SmartDashboard.putNumber("Target Arm Pos", rotatorPos);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
    }
}
