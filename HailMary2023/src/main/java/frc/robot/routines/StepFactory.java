package frc.robot.routines;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.arm.LowerJointConstants;
import frc.robot.arm.UpperJointConstants;

public class StepFactory 
{

    private DoubleSolenoid mLeftFingerSolenoid;
    private DoubleSolenoid mRightFingerSolenoid;
    private DoubleSolenoid mArmClamp;
    private CANSparkMax mWristMotor;
    private Encoder mWristEncoder;
    
    private SparkMaxPIDController mLowerJointPIDController;
    private SparkMaxPIDController mUpperJointPIDController;
  


    private TrapezoidProfile.Constraints mLowerJointConstraints;
    private TrapezoidProfile.Constraints mUpperJointConstraints;
    private TrapezoidProfile.Constraints mUpperJointConstraintsForBigDownMovements;
    
    private AbsoluteEncoder mLowerJointEncoder;
    private AbsoluteEncoder mUpperJointEncoder;
    
    public ClawStepFactory ClawStepsFactory;
    public TimeDelayStepFactory TimeDelayStepsFactory;
    public ClampStepFactory ClampStepsFactory;
    public LowerJointStepFactory LowerJointStepsFactory;
    public UpperJointStepFactory UpperJointStepsFactory;
    public TwistStepFactory TwistStepsFactory;

    public StepFactory(
       DoubleSolenoid pLeftFingerSolenoid,
        DoubleSolenoid pRightFingerSolenoid,
        DoubleSolenoid pArmClamp,
        CANSparkMax pWristMotor,
        Encoder pWristEncoder,        
        SparkMaxPIDController pLowerJointPIDController,
        SparkMaxPIDController pUpperJointPIDController,      
        TrapezoidProfile.Constraints pLowerJointConstraints,
        TrapezoidProfile.Constraints pUpperJointConstraints,
        TrapezoidProfile.Constraints pUpperJointConstraintsForBigDownMovements,
        AbsoluteEncoder pUpperJointEncoder,
        AbsoluteEncoder pLowerJointEncoder)
    {
        mLeftFingerSolenoid = pLeftFingerSolenoid;
        mRightFingerSolenoid = pRightFingerSolenoid;
        mArmClamp = pArmClamp;
        mWristMotor = pWristMotor;
        mWristEncoder = pWristEncoder;
        mLowerJointPIDController = pLowerJointPIDController;
        mUpperJointPIDController = pUpperJointPIDController;      
        mLowerJointConstraints = pLowerJointConstraints;
        mUpperJointConstraints = pUpperJointConstraints;
        mUpperJointConstraintsForBigDownMovements = pUpperJointConstraintsForBigDownMovements;
        mUpperJointEncoder = pUpperJointEncoder;
        mLowerJointEncoder = pLowerJointEncoder;

        ClawStepsFactory = new ClawStepFactory();
        TimeDelayStepsFactory = new TimeDelayStepFactory();
        ClampStepsFactory = new ClampStepFactory();
        LowerJointStepsFactory = new LowerJointStepFactory();
        UpperJointStepsFactory = new UpperJointStepFactory();
        TwistStepsFactory = new TwistStepFactory();
    }

    public class ClawStepFactory
    {
        public ClawStep CreateClawStep(String pName, boolean pShouldClose)
        {
          return new ClawStep(pName, null, mLeftFingerSolenoid, mRightFingerSolenoid, pShouldClose);
        }
      
        public ClawStep CreateOpenClawStep(String pName)
        {
          return CreateClawStep(pName, false) ;
        }
      
      
        public ClawStep CreateCloseClawStep(String pName)
        {
          return CreateClawStep(pName, true) ;
        }
    }

    public class TimeDelayStepFactory
    {  
        public TimeDelayStep CreatePauseBeforeUntwistStep(String pName)
        {
            return new TimeDelayStep(pName, null, 750);
        }

        public TimeDelayStep CreatePauseBeforeUpperArmGoesToScoreStep(String pName)
        {
            return new TimeDelayStep(pName, null, 400);
        }
    }

    public class ClampStepFactory
    {
  
        public ClampStep CreateReleaseClampStep(String pName)
        {
            return CreateClampStep(pName, false);
        }
    
        public ClampStep CreateOpenClampStep(String pName)
        {
            return CreateClampStep(pName, true);
        }
    
        private ClampStep CreateClampStep(String pName, boolean pShouldClose)
        {
            return new  ClampStep(pName, null, mArmClamp, pShouldClose);
        }
    }

    public class UpperJointStepFactory
    {
        public ArmMovementComponentStep CreateUpperDownToGrabLocationStep()
        {
          return new ArmMovementComponentStep("UpperDownToGrabLocationStep", null, mUpperJointConstraints, UpperJointConstants.GRAB_LOCATION, mUpperJointPIDController, mUpperJointEncoder, 0.02, -0.02);
        }
                
        public ArmMovementComponentStep CreateUpperJointMidConeStep()
        {
            return new ArmMovementComponentStep("UpperJointToMidCone", null, mUpperJointConstraints, UpperJointConstants.MID_CONE, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01);
        }

        public ArmMovementComponentStep CreateUpperToRotationSpotStep()
        {
            return new ArmMovementComponentStep("UpperToRotationSpotStep", null, mUpperJointConstraints, UpperJointConstants.ROTATION_SPOT, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01);
        }

        public ArmMovementComponentStep CreateUpperIntakingStep()
        {
            return new ArmMovementComponentStep("upperJointBackToIntake", null, mUpperJointConstraintsForBigDownMovements, UpperJointConstants.INTAKING, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01);
        }

        public ArmMovementComponentStep CreateUpperJointTopConeStep()
        {
            return new ArmMovementComponentStep("UpperJointToMidCone", null, mUpperJointConstraints, UpperJointConstants.TOP_CONE, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01);
        }
        
    }

    public class LowerJointStepFactory
    {

        public ArmMovementComponentStep CreateLowerJointRetractionStep()
        {
          ArmMovementComponentStep returnValue = new ArmMovementComponentStep("LowerJointRetration", null,  mLowerJointConstraints, LowerJointConstants.SCOOCHED_BACKWARDS, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01);
          returnValue.SetInitialVelocity(-0.05);
          return returnValue;
        }

        
        public ArmMovementComponentStep CreateLowerDownToGrabLocationStep()
        {
            return new ArmMovementComponentStep("LowerDownToGrabLocationStep", null,  mLowerJointConstraints, LowerJointConstants.GRAB_LOCATION, mLowerJointPIDController, mLowerJointEncoder, 0.02, -0.02);
        } 

  
        public ArmMovementComponentStep CreateLowerJointScoochBackwardsStep(String pName)
        {
          return new ArmMovementComponentStep(pName, null,  mLowerJointConstraints, LowerJointConstants.SCOOCHED_BACKWARDS, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01);
        } 
      
        public ArmMovementComponentStep CreateLowerJointMidConeStep()
        {
          return new ArmMovementComponentStep("lowerJointMidCone", null,  mLowerJointConstraints, LowerJointConstants.MID_CONE, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01);
        } 
      
        public ArmMovementComponentStep CreateLowerIntakeStep()
        {
          return new ArmMovementComponentStep("lowerJointIntake", null,  mLowerJointConstraints, LowerJointConstants.INTAKING, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01);
        } 
      
        public ArmMovementComponentStep CreateLowerJointTopConeStep()
        {
            return new ArmMovementComponentStep("lowerJointMidCone", null,  mLowerJointConstraints, LowerJointConstants.TOP_CONE, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01);
        }
    }


    public class TwistStepFactory
    {
        public TwistStep CreateTwistToGrab()
        {
            return new TwistStep("TwistToGrab", null, mWristMotor, mWristEncoder, 45, 3, -3, 0.75);
        }

        public TwistStep CreateTwistToScore()
        {
            return new TwistStep("TwistToGrab", null, mWristMotor, mWristEncoder, 0, 3, -3, 0.25);
        }
    }
}
