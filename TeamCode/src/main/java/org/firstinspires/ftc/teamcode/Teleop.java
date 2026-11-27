package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MainTeleop", group = "Teleop")
public class Teleop extends LinearOpMode {


    static final double DRIVE_TICKS_PER_REV   = 537.7;
    static final double SHOOTER_TICKS_PER_REV = 28.0;
    static final double INTAKE_TICKS_PER_REV  = 300;
    static final double DRIVE_MAX_RPM           = 190;
    static final double SHOOTER_MAX_RPM         = 3900;
    static final double INTAKE_MAX_RPM          = 250;

    static final double TRANSFER_SPEED_MULTIPLIER = 1.0;
    static final double SHOOTER_REVERSE_MAX_RPM = 700;
    static double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");


        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotor transferMotor = hardwareMap.dcMotor.get("transferMotor");
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients p = new PIDFCoefficients(80,0,0,0);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, p);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        double driveMaxTicksPerSec = rpmToTicksPerSec(DRIVE_MAX_RPM, DRIVE_TICKS_PER_REV);
        double intakeMaxTicksPerSec = rpmToTicksPerSec(INTAKE_MAX_RPM, INTAKE_TICKS_PER_REV);

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);



            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            double decrease = 1.0;
            frontLeftMotor.setVelocity(frontLeftPower * driveMaxTicksPerSec);
            backLeftMotor.setVelocity(backLeftPower * driveMaxTicksPerSec);
            frontRightMotor.setVelocity(frontRightPower * decrease * driveMaxTicksPerSec);
            backRightMotor.setVelocity(backRightPower * decrease * driveMaxTicksPerSec);


            if (gamepad2.left_trigger_pressed) {
                intakeMotor.setVelocity(intakeMaxTicksPerSec);
                transferMotor.setPower(1.0);
                shooterMotor.setVelocity(-rpmToTicksPerSec(SHOOTER_REVERSE_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else if (gamepad2.left_bumper){
                intakeMotor.setVelocity(-intakeMaxTicksPerSec);
            } else {
                intakeMotor.setVelocity(0);
            }
            if (gamepad2.b) {
                transferMotor.setPower(TRANSFER_SPEED_MULTIPLIER);
            } else if (gamepad2.x) {
                transferMotor.setPower(-TRANSFER_SPEED_MULTIPLIER);
            } else {
                transferMotor.setPower(0);
            }

            if (gamepad2.right_trigger_pressed) {
                shooterMotor.setVelocity(rpmToTicksPerSec(SHOOTER_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else if (gamepad2.right_bumper) {
                shooterMotor.setVelocity(-rpmToTicksPerSec(SHOOTER_REVERSE_MAX_RPM, SHOOTER_TICKS_PER_REV));
            } else {
                shooterMotor.setVelocity(0);
            }

            telemetry.addData("Shooter RPM", shooterMotor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV);
            telemetry.addData("Intake RPM", intakeMotor.getVelocity() * 60.0 / INTAKE_TICKS_PER_REV);
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("FL RPM", frontLeftMotor.getVelocity() * 60.0 / DRIVE_TICKS_PER_REV);
            telemetry.addData("BL RPM", backLeftMotor.getVelocity() * 60.0 / DRIVE_TICKS_PER_REV);
            telemetry.addData("FR RPM", frontRightMotor.getVelocity() * 60.0 / DRIVE_TICKS_PER_REV);
            telemetry.addData("BR RPM", backRightMotor.getVelocity() * 60.0 / DRIVE_TICKS_PER_REV);
            telemetry.update();
        }
    }
}