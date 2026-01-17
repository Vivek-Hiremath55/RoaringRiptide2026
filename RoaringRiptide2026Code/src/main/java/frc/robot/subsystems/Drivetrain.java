package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


// Adding simulation libraries
import edu.wpi.first.wpilibj.RobotBase;

// Libs for 2D simulation------------------------------------------------------------------------
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Command-based subsystem base
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//--------------------------------------------------------------------------------------------

public class Drivetrain extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final PWMSparkMax m_leftLeader = new PWMSparkMax(1);
  private final PWMSparkMax m_leftFollower = new PWMSparkMax(2);
  private final PWMSparkMax m_rightLeader = new PWMSparkMax(3);
  private final PWMSparkMax m_rightFollower = new PWMSparkMax(4);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  // Simulation visualization-------------------------------------
  private final Field2d m_field = new Field2d();
  private Pose2d m_robotPose = new Pose2d();
  //------------------------------------------------------------------

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);

    //PWMSparkMax does not support "addFollower" so this is the compensation stuff -------------
    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);

    // Add these 2 lines so followers match leaders (PWM style)
    m_leftFollower.setVoltage(leftOutput + leftFeedforward);
    m_rightFollower.setVoltage(rightOutput + rightFeedforward);

    //------------------------------------------------------------------------------------------

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    //Simulation Dashborad Addition-------------------------
    SmartDashboard.putData("Field", m_field);
    //------------------------------------------------------

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  // Simple visual-only pose update for simulation----------------------------------------------
public void updateSimPose(double forwardPercent, double rotationPercent) {
  if (!RobotBase.isSimulation()) return;

  double dx = forwardPercent * 0.05;      // tune if you want faster/slower visual motion
  double dthetaDeg = rotationPercent * 5; // tune rotation feel

  m_robotPose =
      new Pose2d(
          m_robotPose.getX() + dx,
          m_robotPose.getY(),
          m_robotPose.getRotation().plus(Rotation2d.fromDegrees(dthetaDeg)));

  m_field.setRobotPose(m_robotPose);
  }
  //---------------------------------------------------------------------------------------------------
}
