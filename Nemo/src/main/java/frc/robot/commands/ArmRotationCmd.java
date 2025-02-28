package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ArmRotation;
import frc.robot.Constants;

public class ArmRotationCmd extends Command {
    private final ArmRotation armRotation;
    private final XboxController xbox;

    private double armPos;
    private boolean manualArmControl;

    private final SendableChooser<Boolean> manualControlChooser;

    public ArmRotationCmd(ArmRotation armRotation, XboxController xbox) {
        this.armRotation = armRotation;
        addRequirements(this.armRotation);

        this.xbox = xbox;
        manualArmControl = false;

        armPos = armRotation.getArmRotatorPos();

        // Initialize SendableChooser
        manualControlChooser = new SendableChooser<>();
        manualControlChooser.setDefaultOption("Automatic Arm Control", false);
        manualControlChooser.addOption("Manual Arm Control", true);
        SmartDashboard.putData("Arm Control Mode", manualControlChooser);
    }

    @Override
    public void initialize() {
    }

    @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
            // Get the selected control mode from the SendableChooser
            manualArmControl = manualControlChooser.getSelected();

            boolean bPressed = xbox.getBButton();
            boolean rbPressed = xbox.getRightBumperButton();
            SmartDashboard.putBoolean("B Button Pressed", bPressed); // Debugging

            if (bPressed) {
                manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armCoralIntakeFromSourcePos;
                armRotation.setArmRotatorPosition(armPos);
            }

            if (rbPressed) {
                manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armPickUpAlgaeFromLowerReefPos;
                armRotation.setArmRotatorPosition(armPos);
            }


            if (manualArmControl) {
                double axis = -MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
                if (axis != 0) {
                    armPos = MathUtil.clamp(
                        armRotation.getArmRotatorPos() + (axis * Constants.ArmConstants.axis_multiplier),
                        Constants.ArmConstants.ArmMinPos,
                        Constants.ArmConstants.ArmMaxPos
                    );
                    armRotation.setArmRotatorPosition(armPos);
                } else {
                    armRotation.setArmRotatorPosition(armPos); // Maintain last position
                }
            } else if (!bPressed && !rbPressed) {
                // Only move to min position if the arm is actually above it
                if (armRotation.getArmRotatorPos() > Constants.ArmConstants.ArmMinPos) {
                    armRotation.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
                }
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        SmartDashboard.putString("ArmCmd", "Ended");
        if (interrupted) {
            SmartDashboard.putString("ArmCmd", "Interrupted");
        }
    }
}