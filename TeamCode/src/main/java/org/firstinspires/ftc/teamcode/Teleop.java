package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Teleop", group = "RearDrive")
public class Teleop extends CommandOpMode {

    private DriveSubsystem driveSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);

        ArcadeDriveCommand arcadeDriveCommand =
                new ArcadeDriveCommand(
                        driveSubsystem,
                        () -> -gamepad1.left_stick_y,
                        () -> gamepad1.right_stick_x
                );

        // Runs whenever no other command is using the drivetrain.
        driveSubsystem.setDefaultCommand(arcadeDriveCommand);

        telemetry.addData("Status", "Initialized - Rear Wheel Drive");
        telemetry.update();
    }

    @Override
    public void run() {
        // Runs the FTCLib command scheduler.
        super.run();

        telemetry.addData("Status", "Driver: GP1");
        telemetry.addData(
                "Drive",
                "L:%.2f R:%.2f",
                driveSubsystem.getLeftPower(),
                driveSubsystem.getRightPower()
        );
        telemetry.update();
    }
}