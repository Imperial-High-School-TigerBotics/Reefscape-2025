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
            boolean yPressed = xbox.getYButton();
            boolean aPressed = xbox.getAButton();
            boolean xPressed = xbox.getXButton();
            boolean rbPressed = xbox.getRightBumperButton();
            boolean lbPressed = xbox.getLeftBumperButton();
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            //Corral intake from source
            if (bPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armCoralIntakeFromSourcePos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Score Coral L2
            if (yPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armScoreCoralL2Pos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Score Coral L3
            if (xPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armScoreCoralL3Pos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Score Coral L4
            if (aPressed) {
                manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armScoreCoralL4Pos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Score Algae In Processor
            if (rtPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armScoreAlgaeInProcessorPos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Pick Up Algae from Lower Reef
            if (lbPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armPickUpAlgaeFromLowerReefPos;
                armRotation.setArmRotatorPosition(armPos);
            }
            
            //Pick Up Algae from Upper Reef
            if (rbPressed) {
                //manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armPickUpAlgaeFromUpperReefPos;
                armRotation.setArmRotatorPosition(armPos);
            }

            //Score Into Barge
            /*if (ltPressed) {
                manualArmControl = false;
                armPos = Constants.PresetElevatorAndArmConstants.armScoreIntoBargePos;
                armRotation.setArmRotatorPosition(armPos);
            }*/

            // Manual Arm Control
            if (manualArmControl) {
                double axis = -MathUtil.applyDeadband(xbox.getRawAxis(4), Constants.stickDeadband);
                if (axis != 0) {
                    armPos = MathUtil.clamp(
                        armRotation.getArmRotatorPos() + (axis * Constants.ArmConstants.manual_arm_speed),
                        Constants.ArmConstants.ArmMinPos,
                        Constants.ArmConstants.ArmMaxPos
                    );
                    armRotation.setArmRotatorPosition(armPos);
                } else {
                    armRotation.setArmRotatorPosition(armPos); // Maintain last position
                }
            } else if (!bPressed && !rbPressed && !lbPressed && !rtPressed && !ltPressed && !yPressed && !aPressed && !xPressed) {
                // Only move to min position if the arm is actually above it
                    armRotation.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
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