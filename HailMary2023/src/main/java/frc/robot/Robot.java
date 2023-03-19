// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.LowerJointConstants;
import frc.robot.arm.UpperJointConstants;
import frc.robot.routines.AComponentStep;
import frc.robot.routines.ArmMovementComponentStep;
import frc.robot.routines.ClampStep;
import frc.robot.routines.ClawStep;
import frc.robot.routines.Routine;
import frc.robot.routines.RoutineFactory;
import frc.robot.routines.StepPrequisite;
import frc.robot.routines.StepSequence;
import frc.robot.routines.TimeDelayStep;
import frc.robot.routines.TwistStep;
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

import com.revrobotics.SparkMaxRelativeEncoder;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import java.util.ArrayList;
import java.util.Arrays;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private CANSparkMax mLowerJointSparkMax;
  private CANSparkMax mUpperJointSparkMax;
  private AbsoluteEncoder mLowerJointEncoder;
  private AbsoluteEncoder mUpperJointEncoder;
  private SparkMaxPIDController mLowerJointPIDController;
  private SparkMaxPIDController mUpperJointPIDController;
  
  private TrapezoidProfile.Constraints mLowerJointConstraints;
  private TrapezoidProfile.State mLowerJointGoal;
  private TrapezoidProfile.State mLowerJointSetpoint;

  private TrapezoidProfile.Constraints mUpperJointConstraints;
  private TrapezoidProfile.State mUpperJointGoal;
  private TrapezoidProfile.State mUpperJointSetpoint;
  private TrapezoidProfile.Constraints mUpperJointConstraintsForBigDownMovements;
  private TrapezoidProfile.Constraints mLowerJointConstraintsForBigMovements;


  private CANSparkMax mWristMotor;



  private DoubleSolenoid mLeftFingerSolenoid;
  private DoubleSolenoid mDoubleSolenoidB;
  private DoubleSolenoid mRightFingerSolenoid;
  private DoubleSolenoid mArmClamp;


  private Encoder mWristEncoder;
  private PIDController mWristPidController;

  private RoutineFactory mRoutineFactory;

  public void robotInit()
  {

    mWristMotor = new CANSparkMax(45, MotorType.kBrushed);
    mWristMotor.setIdleMode(IdleMode.kBrake);
    mWristEncoder = new Encoder(0, 1);

    mSwerveDrive = SwerveFactory.Create2023Swerve();
    mController = new XboxController(0);

    mPneumaticHub = new PneumaticHub(31);
    //mPneumaticHub.disableCompressor();
    mPneumaticHub.enableCompressorDigital();
    mBottomIntake = new CANSparkMax(16, MotorType.kBrushless);
    mTopIntake = new CANSparkMax(3, MotorType.kBrushless);

    mBottomIntake.restoreFactoryDefaults(false); // If true, burn the flash with the factory default parameters?
    mBottomIntake.setSmartCurrentLimit(20);
    mBottomIntake.burnFlash();

    mTopIntake.restoreFactoryDefaults(false); // If true, burn the flash with the factory default parameters?
    mTopIntake.setSmartCurrentLimit(20);
    mTopIntake.burnFlash();
    
    mLeftFingerSolenoid = mPneumaticHub.makeDoubleSolenoid(12,15);
    mDoubleSolenoidB = mPneumaticHub.makeDoubleSolenoid(10,13);
    mRightFingerSolenoid = mPneumaticHub.makeDoubleSolenoid(11,14);
    mArmClamp = mPneumaticHub.makeDoubleSolenoid(8, 9);
     //mUpperJointSparkMax = InitializeArmJointSparkMax(13, false, 0.95, 0.75, 0.1, new PIDFConfiguration(10, 0, 0, 0), true);
    //mLowerJointSparkMax = InitializeArmJointSparkMax(14, true, 0.48, 0.25, 0.05, new PIDFConfiguration(4.5, 0, 0, 0), false);
   
    mUpperJointSparkMax = InitializeArmJointSparkMax(13, false, 0.85 , 0.75, 0.06, new PIDFConfiguration(10, 0, 0, 0), true);
    mLowerJointSparkMax = InitializeArmJointSparkMax(14, false, 0.28, 0.27, 0.05, new PIDFConfiguration(16, 0, 0, 0), true);
    
    mUpperJointEncoder = mUpperJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    mLowerJointEncoder = mLowerJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    mUpperJointPIDController = mUpperJointSparkMax.getPIDController();
    mLowerJointPIDController = mLowerJointSparkMax.getPIDController();

    
    mUpperJointSetpoint = new TrapezoidProfile.State(mUpperJointEncoder.getPosition(), 0);
    mUpperJointConstraints = new TrapezoidProfile.Constraints(1, 0.25);
    mUpperJointConstraintsForBigDownMovements =  new TrapezoidProfile.Constraints(0.1, 0.05);
    mLowerJointConstraints = new TrapezoidProfile.Constraints(0.5, 0.10);

    
    //mLowerJointConstraintsForBigMovements = new TrapezoidProfile.Constraints(0.3, 0.07);
    mLowerJointSetpoint = new TrapezoidProfile.State(mLowerJointEncoder.getPosition(), 0);
    
    mLowerJointPIDTestTargetPosition = mLowerJointEncoder.getPosition();
    mUpperJointPIDTestTargetPosition = mUpperJointEncoder.getPosition();
    

    

    mIdealEndSpotUpperJoint = mUpperJointEncoder.getPosition();
    mIdealEndSpotLowerJoint = mLowerJointEncoder.getPosition();

    mLowerJointSparkMax.set(0);
    mUpperJointSparkMax.set(0);

    mWristPidController = new PIDController(0.025, 0, 0);

    mRoutineFactory = new RoutineFactory(mLeftFingerSolenoid, mRightFingerSolenoid, mArmClamp, mWristMotor, mWristEncoder, mLowerJointPIDController, mUpperJointPIDController, mLowerJointConstraints, mUpperJointConstraints, mUpperJointConstraintsForBigDownMovements, mUpperJointEncoder, mLowerJointEncoder);
    CreateArmRoutines();
  }

  private double  mLowerJointPIDTestTargetPosition;
  private double mUpperJointPIDTestTargetPosition;

  private void TestArmBasicPID()
  {
    double joystickAmount = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();

    SmartDashboard.putNumber("mLowerJointPIDTestTargetPosition", mLowerJointPIDTestTargetPosition);
    SmartDashboard.putNumber("mUpperJointPIDTestTargetPosition", mUpperJointPIDTestTargetPosition);

    if(mController.getRightBumper())
    {      
      //every 20 ms, move by at most 1/200th of a rotation
      //every second 25/100th of a rotation
      mLowerJointPIDTestTargetPosition += joystickAmount / 200.0;
      mLowerJointPIDController.setReference(mLowerJointPIDTestTargetPosition, ControlType.kPosition);
    }
    else
    {
      mLowerJointSparkMax.set(0);
    }


    if(mController.getLeftBumper())
    {      
      //every 20 ms, move by at most 1/200th of a rotation
      //every second 25/100th of a rotation
      mUpperJointPIDTestTargetPosition += joystickAmount / 200.0;
      mUpperJointPIDController.setReference(mUpperJointPIDTestTargetPosition, ControlType.kPosition);
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }

  }



  private Routine mSmoothRoutine;
  private Routine mTopConeRoutine;

  private void CreateArmRoutines()
  {

    mSmoothRoutine = mRoutineFactory.CreateSmoothRoutine();
    mTopConeRoutine = mRoutineFactory.CreateTopConeRoutine();
  }

  private CANSparkMax InitializeArmJointSparkMax(int pCANID, boolean pIsInverted, double pEncoderOffset, double pForwardLimit, double pReverseLimit, PIDFConfiguration pPIDFConfiguration, boolean pEnableSoftLimit) 
  {
    CANSparkMax sparkMax = new CANSparkMax(pCANID, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setSmartCurrentLimit(40);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setInverted(pIsInverted);
    AbsoluteEncoder absoluteEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setZeroOffset(pEncoderOffset);
    
    SparkMaxPIDController sparkMaxPIDController = sparkMax.getPIDController();
    sparkMaxPIDController.setFeedbackDevice(absoluteEncoder);
    sparkMaxPIDController.setP(pPIDFConfiguration.P());
    sparkMaxPIDController.setI(pPIDFConfiguration.I());
    sparkMaxPIDController.setD(pPIDFConfiguration.D());
    sparkMaxPIDController.setFF(pPIDFConfiguration.F());

    //For now, while trying to figure out the numbers
    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, pEnableSoftLimit);
    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, pEnableSoftLimit);

    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)pForwardLimit);  
    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)pReverseLimit);
    
    sparkMax.burnFlash();
   
    return sparkMax;

  }

  private boolean mAButtonLastPressed = false;
  
  
  private void TestRoutines()
  {
//    mArmClamp.set(Value.kReverse);//opens

    //Routine routine= this.mSmoothRoutine;
    Routine routine= this.mTopConeRoutine;
    if(mController.getAButton())
    {
      if(!mAButtonLastPressed)
      {
        routine.Reset();
        //System.out.println("Reset Called");
      }
      mAButtonLastPressed = true;
      routine.Run();
    }
    else
    {
      mAButtonLastPressed = false;
      mLowerJointSparkMax.set(0);
      mUpperJointSparkMax.set(0);
    }
  }

  private void LogSparkMax(String pName, CANSparkMax pSparkMax, AbsoluteEncoder pAbsoluteEncoder)
  {
    SmartDashboard.putNumber(pName + "Output", pSparkMax.getAppliedOutput());
    if(pAbsoluteEncoder != null)
    {
      SmartDashboard.putNumber(pName + "Encoder", pAbsoluteEncoder.getPosition());
    }
    SmartDashboard.putNumber(pName + "Current", pSparkMax.getOutputCurrent());
  }


  public double WRIST_TICKS_PER_MOTOR_REVOLUTION = 28.0;
  public double WRIST_GEAR_RATIO = (68.0 / 13.0) * (68.0 / 13.0) * (68.0 / 13.0) ;
  public double ConvertWristEncoderToDegrees(double pEncoderTicks)
  {
    //deg = 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
    return 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
  }

  public double ConvertDegreesToWristEncoderTicks(double pDegrees)
  {
    //ticks = (deg / 360) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
    return (pDegrees / 360.0) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
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
    LogSparkMax("LowerJoint", mLowerJointSparkMax, mLowerJointEncoder);
    LogSparkMax("UpperJoint", mUpperJointSparkMax, mUpperJointEncoder);
    LogSparkMax("Wrist", mWristMotor, null);
    if(mWristEncoder!=null)
    {
      SmartDashboard.putNumber("Encoder Distance: ", mWristEncoder.getDistance());
      SmartDashboard.putNumber("Encoder Raw: ", mWristEncoder.getRaw());
      SmartDashboard.putNumber("WristAngleDegrees:", ConvertWristEncoderToDegrees(mWristEncoder.getRaw()));
    }
  }


  
  private void WristPIDToAngle(double pTargetAngle)
  {
    mWristPidController.setSetpoint(pTargetAngle);
    double wristPower = mWristPidController.calculate(ConvertWristEncoderToDegrees(mWristEncoder.getRaw()));
    mWristMotor.set(wristPower);
  }

  private void TestWristBasicPID()
  {
    double targetAngle = 0;
    if(mController.getRightBumper())
    {
      if(mController.getBButton())
      {
        targetAngle = 60;
      }
      else if(mController.getXButton())
      {
        targetAngle = -60;
      }
      WristPIDToAngle(targetAngle);
    }
    else
    {
      mWristMotor.set(0);
    }

  }

  private void UpperJointSmartMotionUp()
  {
    double kDt = 0.02;

    mUpperJointGoal = new TrapezoidProfile.State(mIdealEndSpotUpperJoint, 0);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(mUpperJointConstraints, mUpperJointGoal, mUpperJointSetpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    mUpperJointSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("UpperJointSetPoint", mUpperJointSetpoint.position);
    SmartDashboard.putNumber("UpperJointSetVelocity", mUpperJointSetpoint.velocity);

    // Send setpoint to offboard controller PID
    mUpperJointPIDController.setReference(mUpperJointSetpoint.position,ControlType.kPosition);
  }

  private void UpperJointSmartMotionDown()
  {
    double kDt = 0.02;

    mUpperJointGoal = new TrapezoidProfile.State(mIdealEndSpotUpperJoint, 0);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(mUpperJointConstraintsForBigDownMovements, mUpperJointGoal, mUpperJointSetpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    mUpperJointSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("UpperJointSetPoint", mUpperJointSetpoint.position);
    SmartDashboard.putNumber("UpperJointSetVelocity", mUpperJointSetpoint.velocity);

    // Send setpoint to offboard controller PID
    mUpperJointPIDController.setReference(mUpperJointSetpoint.position,ControlType.kPosition);
  }

  private double UPPER_JOINT_SCORE_LOCATION = 0.5;
  private double UPPER_JOINT_INTAKE_LOCATION = 0.11;
  private double mIdealEndSpotUpperJoint;
  private void UpperSmartMotionTest()
  {
    if(mController.getLeftBumper())
    {
      SmartDashboard.putNumber("mIdealEndSpotUpperJoint", mIdealEndSpotUpperJoint);
      
      if(mController.getXButton())
      {
        mIdealEndSpotUpperJoint = UPPER_JOINT_SCORE_LOCATION;
        UpperJointSmartMotionUp();
      }    
      else if(mController.getAButton())
      {
        mIdealEndSpotUpperJoint = UPPER_JOINT_INTAKE_LOCATION;
        UpperJointSmartMotionDown();
      }

      
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }
  }


  private double LOWER_JOINT_SCORE_LOCATION = 0.15;
  private double LOWER_JOINT_INTAKE_LOCATION = 0.10;
  private double mIdealEndSpotLowerJoint;
  private void LowerJointSmartMotion()
  {
    
    double kDt = 0.02;
    mLowerJointGoal = new TrapezoidProfile.State(mIdealEndSpotLowerJoint, 0);

      // Create a motion profile with the given maximum velocity and maximum
      // acceleration constraints for the next setpoint, the desired goal, and the
      // current setpoint.
      var profile = new TrapezoidProfile(mLowerJointConstraints, mLowerJointGoal, mLowerJointSetpoint);

      // Retrieve the profiled setpoint for the next timestep. This setpoint moves
      // toward the goal while obeying the constraints.
      mLowerJointSetpoint = profile.calculate(kDt);
      SmartDashboard.putNumber("LowerJointSetPoint", mLowerJointSetpoint.position);
      SmartDashboard.putNumber("LowerJointSetVelocity", mLowerJointSetpoint.velocity);

      // Send setpoint to offboard controller PID
      mLowerJointPIDController.setReference(mLowerJointSetpoint.position,ControlType.kPosition);
      
  }

  private void LowerJointSmartMotionTest()
  {
    

    //mIdealEndSpotUpperJoint = UPPER_JOINT_SCORE_LOCATION;
    //UpperJointSmartMotion();

    if(mController.getRightBumper())
    {
      LowerJointSmartMotion();

      if(mController.getXButton())
      {
        mIdealEndSpotLowerJoint = LOWER_JOINT_SCORE_LOCATION;
      }    
      else if(mController.getAButton())
      {
        mIdealEndSpotLowerJoint = LOWER_JOINT_INTAKE_LOCATION;
      }

      
    }
    else
    {
      mLowerJointSparkMax.set(0);
    }
  }

  private void SimplePIDArmTest()
  {
    if(mController.getLeftBumper())
    {
      if(mController.getAButton())
      {
        mUpperJointPIDController.setReference(0.25, ControlType.kPosition, 0);
      }
      else if(mController.getXButton())
      {
        mUpperJointPIDController.setReference(0.35, ControlType.kPosition, 0);
      }
      else
      {
        mUpperJointSparkMax.set(0);
      }
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }

    mLowerJointSparkMax.set(0);
  }

  private void  BasicArmTests()
  {
    if(mController.getLeftBumper())
    {
      mUpperJointSparkMax.set(-0.3* (mController.getRightTriggerAxis() - mController.getLeftTriggerAxis()));
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }

    if(mController.getRightBumper())
    {
      mLowerJointSparkMax.set(-0.3 * (mController.getRightTriggerAxis() - mController.getLeftTriggerAxis()));
    }
    else
    {
      mLowerJointSparkMax.set(0);
    }

  }


  public void TestClawGrabs()
  {
    mLeftFingerSolenoid.set(mController.getLeftBumper() ? Value.kReverse : Value.kForward);
    mRightFingerSolenoid.set(mController.getRightBumper() ? Value.kReverse : Value.kForward );
    mArmClamp.set(mController.getStartButton() ? Value.kReverse : Value.kForward);
  }

  public void TestLowerJoint()
  {
    mLowerJointSparkMax.set(-0.3 * (mController.getRightTriggerAxis() - mController.getLeftTriggerAxis()));
  }

  public void TestWrist()
  {
    double rawJoystickValue =  -1.0 * mController.getRightX();
    if(Math.abs(rawJoystickValue) < 0.05)
    {
      mWristMotor.set(0);
    }
    else
    {
      double massagedValue = rawJoystickValue * rawJoystickValue * (rawJoystickValue > 0.0 ? 1.0: -1.0);
      mWristMotor.set(massagedValue);
    }
  }

  public void TestWristPLG()
  {
    double joystickVal = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    boolean isPosiive= joystickVal > 0;
    double squaredValue= joystickVal * joystickVal;
    double limit = 18; 
    if(!isPosiive)
    {
      squaredValue *= -1.0;
    }
    if(Math.abs(squaredValue) > 0.02)
    {
      if(isPosiive)
      {
        if(mWristEncoder.getDistance() > limit)
        {
          mWristMotor.set(0);  
        }
        else
        {
          mWristMotor.set(0.1 + squaredValue/5.0);
        }
      }
      else
      {
        if(mWristEncoder.getDistance() < -limit)
        {
          mWristMotor.set(0);
        }
        else
        {
          mWristMotor.set(-0.1 + squaredValue/5.0);
        }
      }
    }
    else
    {
      mWristMotor.set(0);
    }
  }

  private long mLastTimestamp;

  public void teleopPeriodic()
  {
    if(false)//mController.getStartButton())
    {
      BasicArmTests();
      if(mController.getLeftStickButton())
      {
        mArmClamp.set(Value.kForward);//closes
      }
      if(mController.getRightStickButton())
      {
        mArmClamp.set(Value.kReverse);//opens
      }
    }
    else
    {
    mLastTimestamp = System.currentTimeMillis();
  
    // TestClawGrabs();
    //TestWrist();
    //TestWristBasicPID();
    //TestLowerJoint();
   
   
    //testPnuematics();
    //BasicArmTests();
    //TestArmBasicPID();
    //SimplePIDArmTest();
    //UpperSmartMotionTest();
    
    //LowerJointSmartMotionTest();
     //TestRoutines();
     //testFingers();
    //TestWrist();

      //swerveAndIntake();
      //mArmClamp.set(Value.kForward);//closes
    }

     
    swerveAndIntake(); 
    testPnuematics();
    //mHoldRoutine.Run();
    mUpperJointSparkMax.set(0);
    mLowerJointSparkMax.set(0);

  }

  public void testFingers()
  {
    //A was open
    mLeftFingerSolenoid.set(mController.getAButton() ? Value.kForward : Value.kReverse);
    mRightFingerSolenoid.set(mController.getAButton() ? Value.kForward : Value.kReverse);
  }

  public void testPnuematics()
  {
    
      if(mController.getAButton())
      {
        mLeftFingerSolenoid.set(Value.kForward);
      }
      if(mController.getBButton())
      {
        mLeftFingerSolenoid.set(Value.kReverse);
      }
      if(mController.getXButton())
      {
        mDoubleSolenoidB.set(Value.kForward);
      }
      if(mController.getYButton())
      {
        mDoubleSolenoidB.set(Value.kReverse);
      }
      if(mController.getLeftBumper())
      {
        mRightFingerSolenoid.set(Value.kForward);
      }
      if(mController.getRightBumper())
      {
        mRightFingerSolenoid.set(Value.kReverse);
      }
      if(mController.getLeftStickButton())
      {
        mArmClamp.set(Value.kForward);//closes
      }
      if(mController.getRightStickButton())
      {
        mArmClamp.set(Value.kReverse);//opens
      }

  }

  public void testClaw()
  {
    mLeftFingerSolenoid.set(mController.getRightBumper() ? Value.kForward : Value.kReverse);
    mDoubleSolenoidB.set(mController.getLeftBumper() ? Value.kForward : Value.kReverse);
  }

  public void swerveAndIntake()
  {
    mSwerveDrive.StandardSwerveDrive(mController.getLeftX(), mController.getLeftY(), mController.getRightTriggerAxis(), mController.getRightX());
    
    


    if (true)
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
