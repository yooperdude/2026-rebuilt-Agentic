package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.TurretConstants;

/**
 * Turret subsystem (pose-based aiming only).
 *
 * Key frame-of-reference notes:
 * - 0 deg = front of robot
 * - +deg / -deg = left/right around robot center (based on your encoder sign)
 * - mechanical range is limited to [-180, 180]
 * - startup mechanical position is 90 deg
 *
 * We do NOT use direct camera TX for the turret anymore. Instead, we aim from
 * drivetrain pose + known field target coordinates.
 */
public class TurretSubsystem extends SubsystemBase {
    private static final double DEGREES_PER_MOTOR_ROTATION = 12.0; // from your gearing comment: 30:1 => 12 deg per motor rotation
    private static final double MIN_TURRET_ANGLE_DEG = -180.0;
    private static final double MAX_TURRET_ANGLE_DEG = 180.0;
    private static final double STARTING_TURRET_ANGLE_DEG = 90.0;
    private static final double MAX_TURRET_VOLTAGE = 4.0;
    private static final double AT_TARGET_TOLERANCE_DEG = 1.0;

    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID, "rio");
    private final CommandSwerveDrivetrain drivetrainSubsys;

    // Main position controller for turret angle.
    private final PIDController turretController = new PIDController(0.1, 0.0, 0.0026);

    // "Desired turret angle" in robot-relative degrees.
    public double setAngle = STARTING_TURRET_ANGLE_DEG;
    public boolean atTarget;

    private double voltage;
    private double turretFeedForward;
    private double desiredAngleFromPose;

    public TurretSubsystem(CommandSwerveDrivetrain drivetrainSubsys) {
        this.drivetrainSubsys = drivetrainSubsys;

        /*
         * Important startup alignment:
         * You said turret starts at 90 degrees, so we seed the motor position to match.
         * This keeps software angle and physical angle in agreement right at boot.
         */
        turretMotor.setPosition(STARTING_TURRET_ANGLE_DEG / DEGREES_PER_MOTOR_ROTATION);
    }

    /**
     * Returns current turret angle in degrees.
     *
     * This is the direct conversion from motor rotations using your known gearing.
     */
    public double getEncoder() {
        return turretMotor.getRotorPosition().getValueAsDouble() * DEGREES_PER_MOTOR_ROTATION;
    }

    public double getTurretAngleDeg() {
        return clampToTurretLimits(getEncoder());
    }

    public Command changeSetAngle(double deltaDeg) {
        return runOnce(() -> setAngle = clampToTurretLimits(setAngle + deltaDeg));
    }

    public Command setAngle(double setPositionDeg) {
        return runOnce(() -> setAngle = clampToTurretLimits(setPositionDeg));
    }

    public Command testTurret(double voltage) {
        return runOnce(() -> turretMotor.setVoltage(voltage));
    }

    public Command stopTurret() {
        return runOnce(() -> turretMotor.stopMotor());
    }

    /**
     * Useful manual command if you want one button to "snap" to pose aim.
     */
    public Command turretAlignment() {
        return runOnce(() -> setAngle = calculateDesiredAngleFromPose());
    }

    /**
     * Pose-based aiming command.
     *
     * Every scheduler loop:
     * 1) compute desired turret angle from field geometry
     * 2) run PID to drive to that angle
     */
    public Command aimCommand() {
        return run(() -> {
            desiredAngleFromPose = calculateDesiredAngleFromPose();
            runPositionController(desiredAngleFromPose, false);
        });
    }

    /**
     * Pose-based tracking with small rotational feedforward.
     *
     * Feedforward helps the turret "lead" while the chassis is actively rotating.
     */
    public Command track() {
        return run(() -> {
            desiredAngleFromPose = calculateDesiredAngleFromPose();
            runPositionController(desiredAngleFromPose, true);
        });
    }

    /**
     * Pass mode currently uses same pose-tracking behavior.
     * You can customize this later (for example, fixed pass target coordinates).
     */
    public Command passMode() {
        return track();
    }

    /**
     * Default hold command.
     * Holds whatever setAngle currently is.
     */
    public Command setPosition() {
        return run(() -> runPositionController(setAngle, false));
    }

    /**
     * Robot-to-target distance in feet (for dashboard and future tuning).
     * Watch variable distanceMeters as you are calling it as feet on the dashboard. You can change this to feet in the code if you want, just remember to update the variable name for clarity.
     */
    public double getDistanceToTarget() {
        Pose2d robotPose = drivetrainSubsys.getState().Pose;
        Translation2d targetPosition = getTargetPose();
        double distanceMeters = robotPose.getTranslation().getDistance(targetPosition);
        return Units.metersToFeet(distanceMeters);
    }

    /**
     * Chooses the field target from alliance.
     *
     * This delegates to drivetrain so there is one place in code that defines
     * alliance-based target coordinates.
     */
    public Translation2d getTargetPose() {
        return drivetrainSubsys.getTargetPose();
    }

    /**
     * Core geometry step for pose-based aiming.
     *
     * Math breakdown:
     * - atan2(dy, dx) gives the field-facing angle from robot to target.
     * - subtract robot heading to convert field-relative angle -> robot-relative.
     * - normalize to [-180, 180] and clamp to turret limits.
     */
    public double calculateDesiredAngleFromPose() {
        Pose2d robot = drivetrainSubsys.getState().Pose;
        Translation2d target = getTargetPose();

        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robot.getRotation().getDegrees();
        double robotRelativeTargetDeg = normalizeToSigned180(fieldAngleDeg - robotHeadingDeg);

        return clampToTurretLimits(robotRelativeTargetDeg);
    }

    /**
     * Small helper term to counteract chassis rotation.
     *
     * Scheduler runs ~50 Hz, so dividing deg/s by 50 gives approx deg/loop.
     */
    public double turnFeedforward() {
        turretFeedForward = Math.toDegrees(drivetrainSubsys.getChassisSpeeds().omegaRadiansPerSecond) / 50.0;
        return turretFeedForward;
    }

    private void runPositionController(double requestedAngleDeg, boolean applyFeedforward) {
        double feedforwardDeg = applyFeedforward ? turnFeedforward() : 0.0;
        double finalSetpointDeg = clampToTurretLimits(requestedAngleDeg + feedforwardDeg);
        double currentAngleDeg = getTurretAngleDeg();

        voltage = turretController.calculate(currentAngleDeg, finalSetpointDeg);
        voltage = clamp(voltage, -MAX_TURRET_VOLTAGE, MAX_TURRET_VOLTAGE);

        turretMotor.setVoltage(voltage);

        setAngle = finalSetpointDeg;
        atTarget = Math.abs(finalSetpointDeg - currentAngleDeg) <= AT_TARGET_TOLERANCE_DEG;
    }

    private double clampToTurretLimits(double angleDeg) {
        return clamp(angleDeg, MIN_TURRET_ANGLE_DEG, MAX_TURRET_ANGLE_DEG);
    }

    /**
     * Wrap any angle into [-180, 180].
     * This is convenient when subtracting headings.
     */
    private double normalizeToSigned180(double angleDeg) {
        double wrapped = angleDeg % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        }
        if (wrapped <= -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Current Angle Deg", getTurretAngleDeg());
        SmartDashboard.putNumber("Turret Setpoint Deg", setAngle);
        SmartDashboard.putNumber("Turret Desired Pose Angle Deg", desiredAngleFromPose);
        SmartDashboard.putNumber("Turret Controller Voltage", voltage);
        SmartDashboard.putNumber("Turret Feedforward DegPerLoop", turretFeedForward);
        SmartDashboard.putBoolean("Turret At Target", atTarget);
        SmartDashboard.putNumber("Turret Distance To Target (ft)", getDistanceToTarget());
    }
}
