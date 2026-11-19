package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Auto", group = "Autonomous")
public class auto extends LinearOpMode {

    // Pedro Visualizer coordinates (inches and degrees).
    private static final double START_X = 53.1025;
    private static final double START_Y = 11.4963;
    private static final double START_HEADING_DEG = 90.0;

    private static final double END_X = 5.4818;
    private static final double END_Y = 96.6574;
    private static final double END_HEADING_DEG = 0.0;

    // ---- TUNE THESE ON THE FIELD ----
    private static final double PATH_TIME_SEC = 2.8;   // how long the drive lasts (visualizer says 2.8 s)
    private static final double DRIVE_POWER = 0.8;     // 0..1 speed along the path
    private static final double RAMP_UP_SEC = 0.3;     // soft start so wheels don't slip
    private static final double STRAFE_BOOST = 1.1;    // same strafe compensation as your TeleOp

    private static final double HEADING_KP = 0.02;     // turn power per degree of error
    private static final double HEADING_MAX_POWER = 0.5;
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // Same names and directions as your TeleOp (right side names are swapped there on purpose).
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrakeBehavior();
        setRunWithoutEncoders();

        // GoBilda Pinpoint (configured as "imu" in the hardware map).
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.setOffsets(-18.4, -179.6, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // robot must be still during init

        telemetry.addLine("Timer path: start -> end with linear heading");
        telemetry.addData("Start", "(%.2f, %.2f, %.0f deg)", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("End", "(%.2f, %.2f, %.0f deg)", END_X, END_Y, END_HEADING_DEG);
        telemetry.addLine("Place robot at START, keep it still, then press play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // Tell the Pinpoint where the robot is on the field. Heading is now in field degrees.
        odo.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING_DEG));

        runPath();
        stopDrive();

        // Settle on the final heading.
        holdHeading(END_HEADING_DEG, 1.5);
        stopDrive();

        odo.update();
        telemetry.addLine("Auto finished.");
        telemetry.addData("Final heading", "%.1f deg", getHeadingDegrees());
        telemetry.addData("Pinpoint x", "%.1f in", odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint y", "%.1f in", odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.update();
    }

    /** Timed straight-line drive with heading interpolating from start to end. */
    private void runPath() {
        double fieldAngleRad = Math.atan2(END_Y - START_Y, END_X - START_X);
        double headingChange = normalizeDegrees(END_HEADING_DEG - START_HEADING_DEG);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < PATH_TIME_SEC) {
            double elapsed = timer.seconds();
            double t = elapsed / PATH_TIME_SEC; // 0..1 progress

            double targetHeading = START_HEADING_DEG + headingChange * t;
            double headingDeg = getHeadingDegrees();
            double headingRad = Math.toRadians(headingDeg);

            // Direction to the end point, expressed in the robot's current frame.
            double rel = fieldAngleRad - headingRad;
            double speed = DRIVE_POWER * Math.min(1.0, elapsed / RAMP_UP_SEC);
            double forward = Math.cos(rel) * speed;
            double strafe = -Math.sin(rel) * speed * STRAFE_BOOST; // +strafe = right

            double rx = headingPower(targetHeading, headingDeg);
            setMecanumPower(forward, strafe, rx);

            telemetry.addData("t", "%.2f / %.2f s", elapsed, PATH_TIME_SEC);
            telemetry.addData("Target / actual heading", "%.0f / %.0f", targetHeading, headingDeg);
            telemetry.addData("Pinpoint x", "%.1f in", odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint y", "%.1f in", odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    /** Rotates in place until the heading is within tolerance (or time runs out). */
    private void holdHeading(double targetDeg, double maxSeconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < maxSeconds) {
            double headingDeg = getHeadingDegrees();
            double error = normalizeDegrees(targetDeg - headingDeg);
            if (Math.abs(error) <= HEADING_TOLERANCE_DEG) return;
            setMecanumPower(0, 0, headingPower(targetDeg, headingDeg));
        }
    }

    /** P controller. Positive rx turns clockwise, positive heading is counterclockwise. */
    private double headingPower(double targetDeg, double currentDeg) {
        double error = normalizeDegrees(targetDeg - currentDeg);
        double rx = -error * HEADING_KP;
        return Math.max(-HEADING_MAX_POWER, Math.min(HEADING_MAX_POWER, rx));
    }

    /** Same mecanum math as your TeleOp. forward +, strafe + = right, rx + = clockwise. */
    private void setMecanumPower(double forward, double strafe, double rx) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rx), 1.0);
        frontLeftMotor.setPower((forward + strafe + rx) / denominator);
        backLeftMotor.setPower((forward - strafe + rx) / denominator);
        frontRightMotor.setPower((forward - strafe - rx) / denominator);
        backRightMotor.setPower((forward + strafe - rx) / denominator);
    }

    private double getHeadingDegrees() {
        odo.update(); // refresh Pinpoint data on every read
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    private double normalizeDegrees(double degrees) {
        double normalized = degrees;
        while (normalized > 180.0) normalized -= 360.0;
        while (normalized < -180.0) normalized += 360.0;
        return normalized;
    }

    private void setBrakeBehavior() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setRunWithoutEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}