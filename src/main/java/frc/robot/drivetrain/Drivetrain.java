/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import static com.ctre.phoenix.motorcontrol.ControlMode.Velocity;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2363.utilities.HelixMath;
import com.team319.models.BobTalonSRX;
import com.team319.models.LeaderBobTalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

  private static Drivetrain INSTANCE = new Drivetrain();

  /**
   * @return the singleton instance of the Drivetrain subsystem
   */
  public static Drivetrain getDrivetrain() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private final double WHEEL_DIAMETER_IN_INCHES = 4;
  private final int ENCODER_TICKS_PER_REVOLUTION = (int) (480 * 42.0/48.0);
  public static final double MAX_VELOCITY_IN_FPS = 10;

  private static final int VELOCITY_CONTROL_SLOT = 0;

  //  Competition & Practice Bot  Talon Masters with Victors as Slaves.
  private BaseMotorController rightSlave1 = new BobTalonSRX(11);
  private BobTalonSRX rightSlave2 = new BobTalonSRX(10);
  private BaseMotorController leftSlave1 = new BobTalonSRX(24);
  private BaseMotorController leftSlave2 = new BobTalonSRX(25);

  private LeaderBobTalonSRX left = new LeaderBobTalonSRX(23, leftSlave1, leftSlave2);
  private LeaderBobTalonSRX right = new LeaderBobTalonSRX(12, rightSlave1, rightSlave2);

  //private PowerDistributionPanel pdp = new PowerDistributionPanel();
  private PigeonIMU pigeon = new PigeonIMU(rightSlave2);
  private Camera frontCamera = new Camera("limelight-front");

  private Drivetrain() {
    setPIDFValues();
    setBrakeMode(Brake);
    setupSensors();
    setupLogs();

    left.setSensorPhase(false);
    right.setSensorPhase(false);
    left.setInverted(true);
    right.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new SampleDrive());
  }

  public void setRawPercentOutput(double leftPercent, double rightPercent) {
    left.set(ControlMode.PercentOutput, leftPercent);
    right.set(ControlMode.PercentOutput, rightPercent);
  }

  public void setPercentOutput(double leftPercent, double rightPercent) {
    setVelocityOutput(leftPercent * MAX_VELOCITY_IN_FPS, rightPercent * MAX_VELOCITY_IN_FPS);
  }

  public void setVelocityOutput(double leftVelocity, double rightVelocity) {
    left.set(Velocity, HelixMath.convertFromFpsToTicksPer100Ms(leftVelocity, WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION));
    right.set(Velocity, HelixMath.convertFromFpsToTicksPer100Ms(rightVelocity, WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION));
  }

  private void setPIDFValues() {
    double kF = 1.25;
    double kP = 1; //5
    double kI = 0.01;
    double kD = 0;
    left.configPIDF(VELOCITY_CONTROL_SLOT, kP, kI, kD, kF);
    right.configPIDF(VELOCITY_CONTROL_SLOT, kP, kI, kD, kF);

    left.config_IntegralZone(VELOCITY_CONTROL_SLOT, (int) HelixMath.convertFromFpsToTicksPer100Ms(1, WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION));
    right.config_IntegralZone(VELOCITY_CONTROL_SLOT, (int) HelixMath.convertFromFpsToTicksPer100Ms(1, WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION));
  }

  private void setupSensors() {
    left.configPrimaryFeedbackDevice(FeedbackDevice.QuadEncoder);
    right.configPrimaryFeedbackDevice(FeedbackDevice.QuadEncoder);
  }

  private void setBrakeMode(NeutralMode neutralMode) {
    left.setNeutralMode(neutralMode);
    right.setNeutralMode(neutralMode);
  }

  private void setupLogs() {
    // HelixLogger.getInstance().addDoubleSource("TOTAL CURRENT", pdp::getTotalCurrent);
    // HelixLogger.getInstance().addDoubleSource("DT LM Current", left::getOutputCurrent);
    // HelixLogger.getInstance().addDoubleSource("DT RM Current", right::getOutputCurrent);

    // //  This logging format should work for Talons OR Victor SLAVES.
    // HelixLogger.getInstance().addDoubleSource("DT LS1 Current", () -> pdp.getCurrent(LEFT_SLAVE_1_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT LS2 Current", () -> pdp.getCurrent(LEFT_SLAVE_2_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT RS1 Current", () -> pdp.getCurrent(RIGHT_SLAVE_1_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT RS2 Current", () -> pdp.getCurrent(RIGHT_SLAVE_1_PDP));

    // HelixLogger.getInstance().addDoubleSource("PIGEON HEADING", () -> this.getYaw());

    // HelixLogger.getInstance().addDoubleSource("DRIVETRAIN LEFT Voltage", left::getMotorOutputVoltage);
    // HelixLogger.getInstance().addIntegerSource("DRIVETRAIN LEFT Velocity", this::getLeftVelocity);
    // HelixLogger.getInstance().addDoubleSource("DRIVETRAIN RIGHT Voltage", right::getMotorOutputVoltage);
    // HelixLogger.getInstance().addIntegerSource("DRIVETRAIN RIGHT Velocity", this::getRightVelocity);
  }

  public void resetEncoders() {
    left.getSensorCollection().setQuadraturePosition(0, 0);
    right.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public void resetHeading() {
    pigeon.setYaw(0.0);
  }

  public double getHeading() {
    double [] ypr = {0, 0, 0};
    pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getLeftVelocity() {
      return HelixMath.convertFromTicksPer100MsToFps(left.getSelectedSensorVelocity(), WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION);
  }

  public double getRightVelocity() {
    return HelixMath.convertFromTicksPer100MsToFps(right.getSelectedSensorVelocity(), WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION);
  }

  public double getLeftPosition() {
    return HelixMath.convertFromTicksToFeet(left.getSelectedSensorPosition(), WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION);
  }

  public double getRightPosition() {
    return  HelixMath.convertFromTicksToFeet(right.getSelectedSensorPosition(), WHEEL_DIAMETER_IN_INCHES, ENCODER_TICKS_PER_REVOLUTION);
  }

  public Camera getFrontCamera() {
    return frontCamera;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon Yaw", getHeading());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Left Distance", getLeftPosition());
    SmartDashboard.putNumber("Right Distance", getRightPosition());
  }
}
