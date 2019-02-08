/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax s_motor;
  private CANPIDController s_pidController;
  private CANEncoder s_encoder;
  private WPI_TalonSRX t_motor;
  public SendableChooser<Integer> motorType;
  public int kDeviceID, slotID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kSetPoint, kRampRate;
  public int kCurrLim;
  public boolean positionControl;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    kDeviceID = 1; // CAN ID
    slotID = 0;    // PID slot
    kSetPoint = 0; // Target
    kCurrLim = 30;  // Amps
    kRampRate = 0;  // Seconds to max
    positionControl = false;

    // PID coefficients
    kP = 0.5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1; 
    
    motorType.setDefaultOption("Spark Max", 1);
    motorType.addOption("Talon SRX", 2);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    SmartDashboard.putNumber("Device ID", kDeviceID);
    SmartDashboard.putNumber("Slot ID", slotID);
    SmartDashboard.putNumber("Set Point", kSetPoint);
    SmartDashboard.putNumber("Current Limit", kCurrLim);
    SmartDashboard.putNumber("Ramp Rate", kRampRate);
    SmartDashboard.putBoolean("Position Control", positionControl);  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is run when auton is started up and should be
   * used for any initialization code.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is run when auton is started up and should be
   * used for any initialization code.
   */
  @Override
  public void teleopInit() {
    int deviceID = (int) SmartDashboard.getNumber("Device ID", kDeviceID);
    slotID = (int) SmartDashboard.getNumber("Slot ID", 0);

    if(motorType.getSelected() == 1) {
      try {
        // initialize motor
        if (deviceID == kDeviceID) {s_motor = new CANSparkMax(kDeviceID, MotorType.kBrushless);}
        else {s_motor = new CANSparkMax(deviceID, MotorType.kBrushless);}

        s_motor.setRampRate(kRampRate);
        s_motor.setSmartCurrentLimit(kCurrLim);

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        s_pidController = s_motor.getPIDController();

        // Encoder object created to display position values
        s_encoder = s_motor.getEncoder();

        // set PID coefficients
        s_pidController.setP(kP, slotID);
        s_pidController.setI(kI, slotID);
        s_pidController.setD(kD, slotID);
        s_pidController.setIZone(kIz, slotID);
        s_pidController.setFF(kFF, slotID);
        s_pidController.setOutputRange(kMinOutput, kMaxOutput, slotID);
      } catch(Exception e) { System.out.println("Motor Controller is not a Spark Max"); }
    }
    else {
      try {
        if (deviceID == kDeviceID) {t_motor = new WPI_TalonSRX(kDeviceID);}
        else {t_motor = new WPI_TalonSRX(deviceID);}
        
        t_motor.configClosedloopRamp(kRampRate);
        t_motor.configPeakCurrentLimit(kCurrLim);

        // Encoder object created to display position values
        t_motor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

        // set PID coefficients
        t_motor.selectProfileSlot(slotID, 0); //(int slotIdx, int pidIdx) pidIdx should be 0
			  t_motor.config_kF(slotID, kFF);     //(int slotIdx, double value, int timeoutMs)
			  t_motor.config_kP(slotID, kP);
			  t_motor.config_kI(slotID, kI);
			  t_motor.config_kD(0, kD);
			  t_motor.config_IntegralZone(0, (int) kIz);
        t_motor.configPeakOutputForward(kMaxOutput);
        t_motor.configPeakOutputReverse(kMinOutput);
        
      } catch (Exception e) { System.out.println("Motor Controller is not a Talon SRX"); }
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", kP);
    double i = SmartDashboard.getNumber("I Gain", kI);
    double d = SmartDashboard.getNumber("D Gain", kD);
    double iz = SmartDashboard.getNumber("I Zone", kIz);
    double ff = SmartDashboard.getNumber("Feed Forward", kFF);
    double max = SmartDashboard.getNumber("Max Output", kMaxOutput);
    double min = SmartDashboard.getNumber("Min Output", kMinOutput);
    double setPoint = SmartDashboard.getNumber("Set Point", kSetPoint);

    int currLim = (int) SmartDashboard.getNumber("Current Limit", kCurrLim);
    double rampRate = SmartDashboard.getNumber("Ramp Rate", kRampRate);
    if(motorType.getSelected() == 1) {
      try {
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { s_pidController.setP(p, slotID); kP = p; }
        if((i != kI)) { s_pidController.setI(i, slotID); kI = i; }
        if((d != kD)) { s_pidController.setD(d, slotID); kD = d; }
        if((iz != kIz)) { s_pidController.setIZone(iz, slotID); kIz = iz; }
        if((ff != kFF)) { s_pidController.setFF(ff, slotID); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          s_pidController.setOutputRange(min, max, slotID); 
          kMinOutput = min; kMaxOutput = max;
        }

        if((currLim != kCurrLim)) { s_motor.setSmartCurrentLimit(currLim); }
        if((rampRate != kRampRate)){ s_motor.setRampRate(rampRate); }

        positionControl = SmartDashboard.getBoolean("Position Control", false);

        if (positionControl) {
          if((setPoint != kSetPoint)) {
            s_pidController.setReference(setPoint, ControlType.kPosition);
            SmartDashboard.putNumber("Set Point", setPoint);
          }
          else {
            s_pidController.setReference(kSetPoint, ControlType.kPosition);
            SmartDashboard.putNumber("Set Point", kSetPoint);
          }
        }
        else {
          if((setPoint != kSetPoint)) {
            s_pidController.setReference(setPoint, ControlType.kVelocity);
            SmartDashboard.putNumber("Set Point", setPoint);
          }
          else {
            s_pidController.setReference(kSetPoint, ControlType.kVelocity);
            SmartDashboard.putNumber("Set Point", kSetPoint);
          }
        }
        
        SmartDashboard.putNumber("Motor Position", s_encoder.getPosition());
        SmartDashboard.putNumber("Motor Velocity", s_encoder.getVelocity());
      } catch(Exception e) { System.out.println("Motor Controller is not a Spark Max"); }
    }
    else {
      try {
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { t_motor.config_kP(slotID, p); kP = p; }
        if((i != kI)) { t_motor.config_kI(slotID, i); kI = i; }
        if((d != kD)) { t_motor.config_kD(slotID, d); kD = d; }
        if((iz != kIz)) { t_motor.config_IntegralZone(slotID, (int) iz); kIz = iz; }
        if((ff != kFF)) { t_motor.config_kF(slotID, ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          t_motor.configPeakOutputForward(max);
          t_motor.configPeakOutputReverse(min); 
          kMinOutput = min; kMaxOutput = max;
        }
        
        
        if((currLim != kCurrLim)) { t_motor.configPeakCurrentLimit(currLim); }
        if((rampRate != kRampRate)){ t_motor.configClosedloopRamp(rampRate); }

        positionControl = SmartDashboard.getBoolean("Position Control", false);

        if (positionControl) {
          if((setPoint != kSetPoint)) {
            t_motor.set(ControlMode.Position, setPoint);
            SmartDashboard.putNumber("Set Point", setPoint);
          }
          else {
            t_motor.set(ControlMode.Position, kSetPoint);
            SmartDashboard.putNumber("Set Point", kSetPoint);
          }
        }
        else {
          if((setPoint != kSetPoint)) {
            t_motor.set(ControlMode.Velocity, setPoint);
            SmartDashboard.putNumber("Set Point", setPoint);
          }
          else {
            t_motor.set(ControlMode.Velocity, kSetPoint);
            SmartDashboard.putNumber("Set Point", kSetPoint);
          }
        }
        
        SmartDashboard.putNumber("Motor Position", t_motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Motor Velocity", t_motor.getSelectedSensorVelocity());
      } catch(Exception e) { System.out.println("Motor Controller is not a Talon SRX"); }
    }
  }

  /**
   * Initialization code for disabled mode should go here.
   */
  @Override
  public void disabledInit() {
    if(motorType.getSelected() == 1) {
      try {
        s_motor.stopMotor();
        s_motor.close();
      } catch (Exception e) { System.out.println("Motor Controller is not a Spark Max"); }
    }
    else {      
      try {
        t_motor.stopMotor();
        t_motor.free();
      } catch (Exception e) { System.out.println("Motor Controller is not a Talon SRX"); }      
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
