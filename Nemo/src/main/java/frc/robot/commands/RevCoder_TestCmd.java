import edu.wpi.first.wpilibj2.command.CommandBase;

public class RevCoder_TestCmd extends CommandBase {
    private final RevCoder_test revCoderTest;

    public RevCoder_TestCmd(RevCoder_test revCoderTest) {
        this.revCoderTest = revCoderTest;
        addRequirements(revCoderTest);
    }

    @Override
    public void initialize() {
        revCoderTest.reset();
    }

    @Override
    public void execute() {
        revCoderTest.updateDashboard();
    }

}