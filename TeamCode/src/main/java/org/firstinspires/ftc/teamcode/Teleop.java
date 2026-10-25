
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop extends LinearOpMode {

    // Verify against the exact goBILDA motor encoder configuration.
    private static final double TICKS_PER_REV = 28.0;
    // Flywheel target. Equal to motor RPM while the pulley ratio is 1.0.
    private static final double TARGET_RPM = 1400.0;
    // External belt ratio ONLY: flywheel pulley teeth / motor pulley teeth.
    // 1.0 assumes equal pulleys.
    private static final double SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN = 1.0;

    // Initial test powers; tune on the completed robot.
    private static final double TRANSFER_FEED_POWER = 0.25;
    private static final double TRANSFER_REVERSE_POWER = 0.25;
    private static final double INTAKE_REVERSE_POWER = 0.40;

    // Verify physical directions. Change an individual value to REVERSE if needed.
    private static final DcMotorSimple.Direction TRANSFER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction INTAKE_DIRECTION =
            DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drivetrain motors
        DcMotor frontLeftMotor =
                hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor =
                hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor =
                hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor =
                hardwareMap.dcMotor.get("backRightMotor");

        // Intake motor
        DcMotor intakeMotor =
                hardwareMap.dcMotor.get("intakeMotor");

        // Transfer motor: drives the rotors that feed the shooter
        DcMotor transferMotor =
                hardwareMap.dcMotor.get("transferMotor");

        // Shooter motor with encoder feedback
        DcMotorEx shooterMotor =
                hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // Reverse right drivetrain motors
        frontRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE);

        // Intake configuration
        intakeMotor.setPower(0.0);
        intakeMotor.setDirection(INTAKE_DIRECTION);
        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Transfer configuration
        transferMotor.setPower(0.0);
        transferMotor.setDirection(TRANSFER_DIRECTION);
        transferMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter configuration
        shooterMotor.setPower(0.0);
        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        double targetTicksPerSecond = TARGET_RPM
                * SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN
                * TICKS_PER_REV / 60.0;

        // IMU setup
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("P1: Drive | P2: LT Intake, B Intake Reverse");
        telemetry.addLine("P2: RB/LB Transfer, RT Shooter");
        telemetry.update();

        try {
            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {

                // =====================================
                // PLAYER 1: FIELD-CENTRIC DRIVETRAIN
                // =====================================

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading)
                        - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading)
                        + y * Math.cos(-botHeading);

                rotX *= 1.1;

                double denominator = Math.max(
                        Math.abs(rotY) + Math.abs(rotX)
                                + Math.abs(rx), 1.0);

                double frontLeftPower =
                        (rotY + rotX + rx) / denominator;
                double backLeftPower =
                        (rotY - rotX + rx) / denominator;
                double frontRightPower =
                        (rotY - rotX - rx) / denominator;
                double backRightPower =
                        (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                // =====================================
                // PLAYER 2: INTAKE
                // LEFT TRIGGER: IN (VARIABLE SPEED)
                // HOLD B: REVERSE TO CLEAR JAMS
                // =====================================

                double intakePower = gamepad2.left_trigger;
                if (gamepad2.b) {
                    intakePower = -INTAKE_REVERSE_POWER;
                }
                intakeMotor.setPower(intakePower);

                // =====================================
                // PLAYER 2: TRANSFER
                // HOLD RB: FEED TOWARD SHOOTER
                // HOLD LB: REVERSE
                // BOTH OR NEITHER: STOP
                // =====================================

                double transferPower = 0.0;
                if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                    transferPower = TRANSFER_FEED_POWER;
                } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferPower = -TRANSFER_REVERSE_POWER;
                }
                transferMotor.setPower(transferPower);

                // =====================================
                // PLAYER 2: SHOOTER (RIGHT TRIGGER)
                // FIXED TARGET: 1400 RPM
                // =====================================

                boolean shooterHeld = gamepad2.right_trigger > 0.05;

                if (shooterHeld) {
                    shooterMotor.setVelocity(targetTicksPerSecond);
                } else {
                    shooterMotor.setPower(0.0);
                }

                // =====================================
                // DRIVER STATION TELEMETRY
                // =====================================

                double measuredRPM = shooterMotor.getVelocity()
                        * 60.0 / TICKS_PER_REV
                        / SHOOTER_MOTOR_TURNS_PER_FLYWHEEL_TURN;

                telemetry.addData("Intake Power",
                        "%.0f%%", intakePower * 100.0);
                telemetry.addData("Transfer Power",
                        "%.0f%%", transferPower * 100.0);
                telemetry.addData("Shooter Target RPM",
                        shooterHeld ? "%.0f" : "OFF",
                        TARGET_RPM);
                telemetry.addData("Shooter Measured RPM",
                        "%.0f", measuredRPM);
                telemetry.update();
            }
        } finally {
            shooterMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
            transferMotor.setPower(0.0);
            frontLeftMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
        }
    }
}
