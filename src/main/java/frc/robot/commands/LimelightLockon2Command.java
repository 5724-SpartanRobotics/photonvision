package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;

public class LimelightLockon2Command extends Command {
    private final DriveTrainSubsystem driveTrainSubsystem;
    private final LimelightLockonSubsystem limelightLockonSubsystem;

    public LimelightLockon2Command(
        DriveTrainSubsystem driveTrain,
        LimelightLockonSubsystem llLockon
    ) {
        super();

        addRequirements(driveTrain, llLockon);
        this.driveTrainSubsystem = driveTrain;
        this.limelightLockonSubsystem = llLockon;
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
