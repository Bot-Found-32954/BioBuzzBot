package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveSubsystem extends SubsystemBase {

    // Must be public and non-final for FTC Dashboard tuning.
    public static double DRIVE_SPEED_MULTIPLIER = 2.0;

    private final DcMotor leftDriveFront;
    private final DcMotor leftDriveBack;
    private final DcMotor rightDriveFront;
    private final DcMotor rightDriveBack;

    private double leftPower;
    private double rightPower;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftDriveFront =
                hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveBack =
                hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveFront =
                hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveBack =
                hardwareMap.get(DcMotor.class, "right_drive_back");

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        leftDriveFront.setZeroPowerBehavior(BRAKE);
        leftDriveBack.setZeroPowerBehavior(BRAKE);
        rightDriveFront.setZeroPowerBehavior(BRAKE);
        rightDriveBack.setZeroPowerBehavior(BRAKE);
    }

    public void arcadeDrive(double forward, double rotate) {
        forward *= DRIVE_SPEED_MULTIPLIER;
        rotate *= DRIVE_SPEED_MULTIPLIER;

        leftPower = forward + rotate;
        rightPower = forward - rotate;

        double maxPower = Math.max(
                Math.abs(leftPower),
                Math.abs(rightPower)
        );

        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        leftDriveFront.setPower(leftPower);
        leftDriveBack.setPower(leftPower);
        rightDriveFront.setPower(rightPower);
        rightDriveBack.setPower(rightPower);
    }

    public void stop() {
        leftPower = 0;
        rightPower = 0;

        leftDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveFront.setPower(0);
        rightDriveBack.setPower(0);
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }
}