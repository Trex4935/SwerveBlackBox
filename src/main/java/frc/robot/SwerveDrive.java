package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// sets values of speed and roation on the field
public class SwerveDrive implements Loggable {
  private static SwerveDrive SINGLE_INSTANCE = new SwerveDrive();
  private double SDxSpeed = 0;
  private double SDySpeed = 0;
  private double SDrotation = 0;
  private boolean SDFieldRelative = Constants.DEFAULT_FIELD_RELATIVE_DRIVE;
  private boolean holdRobotAngleEnabled = Constants.DEFAULT_HOLD_ROBOT_ANGLE;
  private PIDController holdRobotAngleController = new PIDController(Constants.ROBOTHoldAngleKP, 0, 0);
  // sets the point to ?hold the angle?; if nothing is being pressed, robot no move
  public double holdRobotAngleSetpoint = Constants.DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT;
  public double joystickDriveGovernor = Constants.SPEED_GOVERNOR;
  public String NeutralMode = "Brake";
  // sets the wheel locations onto a 2d plane
  public final Translation2d m_frontLeftLocation = new Translation2d(Constants.WHEEL_BASE_METERS / 2,
      Constants.WHEEL_BASE_METERS / 2);
  public final Translation2d m_frontRightLocation = new Translation2d(Constants.WHEEL_BASE_METERS / 2,
      -Constants.WHEEL_BASE_METERS / 2);
  public final Translation2d m_backLeftLocation = new Translation2d(-Constants.WHEEL_BASE_METERS / 2,
      Constants.WHEEL_BASE_METERS / 2);
  public final Translation2d m_backRightLocation = new Translation2d(-Constants.WHEEL_BASE_METERS / 2,
      -Constants.WHEEL_BASE_METERS / 2);
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  public SwerveDriveOdometry m_odometry;

  public static SwerveDrive getInstance() {
    return SINGLE_INSTANCE;
  }

  // gets gyro and motor value to determine position
  public void init() {
    m_odometry = new SwerveDriveOdometry(m_kinematics, SwerveMap.getRobotAngle());
    holdRobotAngleController.disableContinuousInput();
    holdRobotAngleController.setTolerance(Math.toRadians(2));
  }

  /**
   * Method to drive the robot using the following params
   *
   * @param _xSpeed        Speed of the robot in the x direction (forward).
   * @param _ySpeed        Speed of the robot in the y direction (sideways).
   * @param _rot           Angular rate of the robot.
   * @param _fieldRelative Whether the provided x and y speeds are relative to the
   *                       field.
   */
  // gets relative speed/angle/radians/rotation to set its ?position relative to its desired state?
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    if (_rot == 0 && holdRobotAngleEnabled) {
      _rot = holdRobotAngleController.calculate(SwerveMap.getRobotAngle().getRadians(), holdRobotAngleSetpoint);
    } else {
      holdRobotAngleSetpoint = SwerveMap.getRobotAngle().getRadians();
    }
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(
        _fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, SwerveMap.getRobotAngle())
            : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_SPEED_METERSperSECOND);

    SwerveMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
    SwerveMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
    SwerveMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
    SwerveMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);
  }

  /**
   * This ONLY saves speeds. You must also call the drive method
   * gets values of x/y and roation on joysticks
   */
  public void joystickDrive() {
    double x = -Robot.xbox.getLeftY();
    double y = -Robot.xbox.getLeftX();
    double rot = -Robot.xbox.getRightX();
    // converts speed and rotation to m/s
    SDxSpeed = convertToMetersPerSecond(deadband(x)) * Constants.SPEED_GOVERNOR;
    SDySpeed = convertToMetersPerSecond(deadband(y)) * Constants.SPEED_GOVERNOR;
    SDrotation = convertToRadiansPerSecond(deadband(rot)) * Constants.SPEED_GOVERNOR;
    // System.out.println(SDrotation);

  }

  /**
   * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
   * sets all the talons (steer and drive motors) to coast.
   * This allows for easy moving of the robot
   */
  public void setToCoast() {
    // gets sensor velocity to determine if robot breaks or coasts
    if (NeutralMode == "Brake" &&
        Math.abs(SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100 &&
        Math.abs(SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100 &&
        Math.abs(SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100 &&
        Math.abs(SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100) {
      SwerveMap.FrontRightSwerveModule.swerveDisabledInit();
      SwerveMap.BackRightSwerveModule.swerveDisabledInit();
      SwerveMap.FrontLeftSwerveModule.swerveDisabledInit();
      SwerveMap.BackLeftSwerveModule.swerveDisabledInit();
      NeutralMode = "Coast";
    } else {
    }
  }

  /** When we drive around we want the robot to brake... "nuff said"-whoever made the code */
  public void setToBrake() {
    SwerveMap.FrontRightSwerveModule.swerveEnabledInit();
    SwerveMap.BackRightSwerveModule.swerveEnabledInit();
    SwerveMap.FrontLeftSwerveModule.swerveEnabledInit();
    SwerveMap.BackLeftSwerveModule.swerveEnabledInit();
    NeutralMode = "Brake";
  }

  /** Sets the robots speed parameters to zero */
  public void zeroSwerveDrive() {
    SDxSpeed = 0;
    SDySpeed = 0;
    SDrotation = 0;
  }
//conversions to m/s
  private double convertToMetersPerSecond(double _input) {
    return _input * Constants.MAX_SPEED_METERSperSECOND;
  }
//conversions to radians per second 
  private double convertToRadiansPerSecond(double _input) {
    return _input * Constants.MAX_SPEED_RADIANSperSECOND;
  }
//provides a range in where the robot breaks
  private double deadband(double _input) {
    if (Math.abs(_input) <= Constants.XBOXDEADBAND) {
      _input = 0;
    }
    return _input;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(

        SwerveMap.getRobotAngle(),
        SwerveMap.FrontLeftSwerveModule.getState(),
        SwerveMap.FrontRightSwerveModule.getState(),
        SwerveMap.BackLeftSwerveModule.getState(),
        SwerveMap.BackRightSwerveModule.getState());
    // System.out.println("x= " + m_odometry.getPoseMeters().getX() + "
    // y="+m_odometry.getPoseMeters().getY());
    // System.out.println("FL: " +
    // Math.round(SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())
    // + " FR: "
    // +Math.round(SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
    // System.out.println("BL: " +
    // Math.round(SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())
    // + " BR: "
    // +Math.round(SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
  }
//Logging in gyro information on rotation/angle/speed(x and y) relative to the feild
  @Log.Gyro(name = "Robot Angle", rowIndex = 2, columnIndex = 5)
  private AHRS getGyro() {
    return SwerveMap.GYRO;
  }

  public boolean getSDFieldRelative() {
    return SDFieldRelative;
  }

  public void setSDxSpeed(double _input) {
    SDxSpeed = _input;
  }

  public void setSDySpeed(double _input) {
    SDySpeed = _input;
  }

  public void setSDRotation(double _input) {
    SDrotation = _input;
  }

  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 7, height = 1, width = 1)
  public double getSDxSpeed() {
    return SDxSpeed;
  }

  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 8, height = 1, width = 1)
  public double getSDySpeed() {
    return SDySpeed;
  }

  @Log.Dial(rowIndex = 0, columnIndex = 9, height = 1, width = 1)
  public double getSDRotation() {
    return SDrotation;
  }
// ?determines if the robot is oriented to the field?
  @Config.ToggleButton(name = "FieldOriented?", defaultValue = true, rowIndex = 1, columnIndex = 0, height = 1, width = 2)
  public void setSDFieldRelative(boolean _input) {
    SDFieldRelative = _input;
  }

  @Config.ToggleButton(name = "Hold Robot Angle?", defaultValue = true, rowIndex = 0, columnIndex = 0, height = 1, width = 2)
  public void setHoldAngleEnabled(boolean _boolean) {
    holdRobotAngleEnabled = _boolean;
  }
// get current robot angle
  @Log.Dial(name = "Current Robot Angle", min = -180, max = 180, rowIndex = 0, columnIndex = 3)
  public double getRobotAngleDegrees() {
    return SwerveMap.getRobotAngle().getDegrees();
  }

  @Log.Dial(name = "Hold Angle Setpoint", min = -180, max = 180, rowIndex = 0, columnIndex = 4)
  public double getHoldAngleSetpoint() {
    return Math.toDegrees(holdRobotAngleSetpoint);
  }

  public void setHoldRobotAngleSetpoint(double _holdRobotAngleSetpoint) {
    holdRobotAngleSetpoint = Math.toRadians(_holdRobotAngleSetpoint);
  }
//gets robot angle/rotation; resets origonal position to the new position 
  public void resetOdometry() {
    m_odometry.resetPosition(new Pose2d(), SwerveMap.getRobotAngle());
  }

  public void resetOdometry(Pose2d _Pose2d, Rotation2d _Rotation2d) {
    m_odometry.resetPosition(_Pose2d, _Rotation2d);
  }
// returns each motor state (speed and angle) (FL, FR, BL, BR motors)
  @Log.NumberBar(name = "FL Speed", min = -5, max = 5, rowIndex = 2, columnIndex = 4, height = 1, width = 1)
  public double getFrontLeftSpeed() {
    return SwerveMap.FrontLeftSwerveModule.getState().speedMetersPerSecond;
  }

  @Log.Dial(name = "FL Angle", min = -90, max = 90, rowIndex = 2, columnIndex = 3, height = 1, width = 1)
  public double getFrontLeftAngle() {
    return Math.IEEEremainder(SwerveMap.FrontLeftSwerveModule.getState().angle.getDegrees(), 180);
  }

  @Log.NumberBar(name = "FR Speed", min = -5, max = 5, rowIndex = 2, columnIndex = 7, height = 1, width = 1)
  public double getFrontRightSpeed() {
    return SwerveMap.FrontRightSwerveModule.getState().speedMetersPerSecond;
  }

  @Log.Dial(name = "FR Angle", min = -90, max = 90, rowIndex = 2, columnIndex = 8, height = 1, width = 1)
  public double getFrontRightAngle() {
    return Math.IEEEremainder(SwerveMap.FrontRightSwerveModule.getState().angle.getDegrees(), 180);
  }

  @Log.NumberBar(name = "BL Speed", min = -5, max = 5, rowIndex = 3, columnIndex = 4, height = 1, width = 1)
  public double getBackLeftSpeed() {
    return SwerveMap.BackLeftSwerveModule.getState().speedMetersPerSecond;
  }

  @Log.Dial(name = "BL Angle", min = -90, max = 90, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
  public double getBackLeftAngle() {
    return Math.IEEEremainder(SwerveMap.BackLeftSwerveModule.getState().angle.getDegrees(), 180);
  }

  @Log.NumberBar(name = "BR Speed", min = -5, max = 5, rowIndex = 3, columnIndex = 7, height = 1, width = 1)
  public double getBackRightSpeed() {
    return SwerveMap.BackRightSwerveModule.getState().speedMetersPerSecond;
  }

  @Log.Dial(name = "BR Angle", min = -90, max = 90, rowIndex = 3, columnIndex = 8, height = 1, width = 1)
  public double getBackRightAngle() {
    return Math.IEEEremainder(SwerveMap.BackRightSwerveModule.getState().angle.getDegrees(), 180);
  }

  @Log(rowIndex = 0, columnIndex = 5, height = 1, width = 1)
  public double getXPos() {
    return m_odometry.getPoseMeters().getX();
  }

  @Log(rowIndex = 0, columnIndex = 6, height = 1, width = 1)
  public double getYPos() {
    return m_odometry.getPoseMeters().getY();
  }

  @Log.BooleanBox(rowIndex = 1, columnIndex = 5)
  public boolean getGyroInterference() {
    return SwerveMap.GYRO.isMagneticDisturbance();
  }

  @Config.NumberSlider(name = "Governor", defaultValue = .11, min = 0, max = 1, rowIndex = 2, columnIndex = 0, height = 1, width = 2)
  public void setJoystickGovernor(double _input) {
    joystickDriveGovernor = _input;
  }
//resets gyro and odomentry to deault values
  @Config.ToggleButton(name = "ResetGyroAndOdometry", defaultValue = false, rowIndex = 3, columnIndex = 0, height = 1, width = 2)
  public void resetGyroAndOdometry(boolean _input) {
    if (_input) {
      SwerveMap.GYRO.reset();
      holdRobotAngleSetpoint = 0;
      Robot.SWERVEDRIVE.m_odometry.resetPosition(new Pose2d(), SwerveMap.getRobotAngle());
      _input = false;
    } else {
    }
  }

  @Config.ToggleButton(name = "RE-Zero Swerve Angle", defaultValue = false, rowIndex = 4, columnIndex = 0, height = 1, width = 2)
  public void reZeroSwerveDrive(boolean _input) {
    if (_input) {
      SwerveMap.FrontRightSwerveModule.REzeroSwerveAngle();
      SwerveMap.BackRightSwerveModule.REzeroSwerveAngle();
      SwerveMap.FrontLeftSwerveModule.REzeroSwerveAngle();
      SwerveMap.BackLeftSwerveModule.REzeroSwerveAngle();
      _input = false;
    } else {
    }
  }

}
