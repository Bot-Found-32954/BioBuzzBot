package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class ArcadeDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotateSupplier;

    public ArcadeDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier rotateSupplier
    ) {
        this.driveSubsystem = driveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.rotateSupplier = rotateSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(
                forwardSupplier.getAsDouble(),
                rotateSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // This command should run continuously during TeleOp.
        return false;
    }
}