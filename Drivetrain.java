// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final AHRS m_gyroscope = new AHRS();
  private final RamseteController m_controller = new RamseteController();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final DifferentialDriveOdometry m_odom = new DifferentialDriveOdometry(new Rotation2d(m_gyroscope.getAngle()));

  private NetworkTable table;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public Encoder getRight() {
    return m_rightEncoder;
  }
  
  public Encoder getLeft() {
    return m_leftEncoder;
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    m_leftEncoder.setDistancePerPulse(0.001);
    m_rightEncoder.setDistancePerPulse(0.001);
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  } 

  public Pose2d getPos() {
    return m_odom.update(new Rotation2d(m_gyroscope.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void getOutVolts(Double x, Double y) {
    x = 12.0;
    y = 12.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  public void createTables() {
    //NetworkTable.setClientMode();
    //NetworkTable.setTeam(9114);
    //NetworkTable.setClientMode();
    //NetworkTable.setTeam(9114);
    table = NetworkTableInstance.getDefault().getTable("datatable");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");
    NetworkTableEntry cl = table.getEntry("cl");
    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    NetworkTableEntry getpipe = table.getEntry("get-pipe");
    NetworkTableEntry json = table.getEntry("json");
    NetworkTableEntry tclass = table.getEntry("tclass");

    double x = tx.getDouble(0.0);
    double y = tx.getDouble(0.0);
    double v = tx.getDouble(0.0);
    double a = tx.getDouble(0.0);
    double tl2 = tx.getDouble(0.0);
    double cl2 = tx.getDouble(0.0);
    double short2 = tx.getDouble(0.0);
    double long2 = tx.getDouble(0.0);
    double hor = tx.getDouble(0.0);
    double vert = tx.getDouble(0.0);
    double pipe = tx.getDouble(0.0);
    double json2 = tx.getDouble(0.0);
    double class2 = tx.getDouble(0.0);


    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightY", v);
    SmartDashboard.putNumber("LimelightV", a);
    SmartDashboard.putNumber("LimelightTl", tl2);
    SmartDashboard.putNumber("LimelightCl", cl2);
    SmartDashboard.putNumber("LimelightHor", hor);
    SmartDashboard.putNumber("LimelightVert", vert);
    SmartDashboard.putNumber("LimelightPipe", pipe);
    SmartDashboard.putNumber("LimelightJson", json2);
    SmartDashboard.putNumber("LimelightClass", class2);
    SmartDashboard.putNumber("LimelightShort", short2);
    SmartDashboard.putNumber("LimelightLong", long2);
  }

  

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  /*public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }*/

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  /*public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }*/

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  /*public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  /*public void resetGyro() {
    m_gyro.reset();
  }*/

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public AHRS getGyro() {
    return m_gyroscope;
  }

  public RamseteController getRC() {
    return m_controller;
  }


}
