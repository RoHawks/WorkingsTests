// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import robosystems.*;

import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;
import universalSwerve.components.implementations.FalconTranslationSystem;
import universalSwerve.components.implementations.SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously;
import universalSwerve.utilities.PIDFConfiguration;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import java.util.concurrent.CancellationException;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SwerveDrive mSwerveDrive;
  private XboxController mController;
  private PneumaticHub mPneumaticHub;
  private CANSparkMax mBottomIntake;
  private CANSparkMax mTopIntake;
  private Intake mIntake;
  private Claw mClaw;

  public void robotInit()
  {
    mSwerveDrive= SwerveFactory.Create2023Swerve();
    mController = new XboxController(0);

    mPneumaticHub = new PneumaticHub(31);
    
    mBottomIntake = new CANSparkMax(16, MotorType.kBrushless);
    mTopIntake = new CANSparkMax(3, MotorType.kBrushless);

    mBottomIntake.restoreFactoryDefaults(true); // If true, burn the flash with the factory default parameters?
    mBottomIntake.setSmartCurrentLimit(20);
    mBottomIntake.burnFlash();

    mTopIntake.restoreFactoryDefaults(true); // If true, burn the flash with the factory default parameters?
    mTopIntake.setSmartCurrentLimit(20);
    mTopIntake.burnFlash();


    // mIntake = new Intake(new CANSparkMax(13, MotorType.kBrushless), 
    //                      new CANSparkMax(16, MotorType.kBrushless),
    //                      mPneumaticHub.makeDoubleSolenoid(12, 15));
    
    // mClaw = new Claw(mPneumaticHub.makeDoubleSolenoid(10, 13),
    //                  mPneumaticHub.makeDoubleSolenoid(11, 14));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    
  }

  public void teleopPeriodic()
  {
    //mSwerveDrive.StandardSwerveDrive(mController.getLeftX(), mController.getLeftY(), , mController.getRightX());
    mSwerveDrive.StandardSwerveDrive(mController.getLeftX(), mController.getLeftY(), mController.getRightTriggerAxis(), mController.getRightX());
    
    if (mController.getAButton())
    {
       mBottomIntake.set(-0.5);
        mTopIntake.set(-1.0);
    }
    else
    {
      mBottomIntake.set(0);
      mTopIntake.set(0);
    }

  }
  

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
 
  } 
    

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
