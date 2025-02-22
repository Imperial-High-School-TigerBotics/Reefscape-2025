package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RevCoder_test;

public class RevCoder_TestCmd extends Command {
    private final RevCoder_test revCoderTest;

    public RevCoder_TestCmd(RevCoder_test revCoderTest) {
        this.revCoderTest = revCoderTest;
        addRequirements(revCoderTest);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        revCoderTest.updateDashboard();
    }

}