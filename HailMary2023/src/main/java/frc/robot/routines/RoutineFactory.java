package frc.robot.routines;

import java.util.Arrays;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

public class RoutineFactory 
{
    private StepFactory mStepFactory;

    public RoutineFactory( DoubleSolenoid pLeftFingerSolenoid,
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
        mStepFactory = new StepFactory(
            pLeftFingerSolenoid,
            pRightFingerSolenoid,
            pArmClamp,
            pWristMotor,
            pWristEncoder,        
            pLowerJointPIDController,
            pUpperJointPIDController,      
            pLowerJointConstraints,
            pUpperJointConstraints,
            pUpperJointConstraintsForBigDownMovements,
            pUpperJointEncoder,
            pLowerJointEncoder
        );

    }


    public Routine CreateSmoothRoutine()
    {
      
        ClampStep releaseClamp = mStepFactory.ClampStepsFactory.CreateReleaseClampStep("InitalClampRelease");
        ClampStep closeClamp = mStepFactory.ClampStepsFactory.CreateOpenClampStep("FinalClampRelease");

        TimeDelayStep pauseBeforeUpperArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUpperArmGoesToScoreStep("pauseBeforeUpperArmGoesToScore");
        TimeDelayStep pauseBeforeUntwist = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUntwistStep("PauseBeforeUntwist");
        

        ClawStep initialOpen = mStepFactory.ClawStepsFactory.CreateOpenClawStep("InitialOpen");
        ClawStep grab = mStepFactory.ClawStepsFactory.CreateCloseClawStep("Grab");
        ClawStep releaseToScore = mStepFactory.ClawStepsFactory.CreateOpenClawStep("ReleaseToScore");


        TwistStep twistToGrab = mStepFactory.TwistStepsFactory.CreateTwistToGrab();
        TwistStep twistToScore = mStepFactory.TwistStepsFactory.CreateTwistToScore();

        ArmMovementComponentStep upperJointUpToRotationSpot = mStepFactory.UpperJointStepsFactory.CreateUpperToRotationSpotStep();
        ArmMovementComponentStep upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStep();
        ArmMovementComponentStep upperJointUpToScore = mStepFactory.UpperJointStepsFactory.CreateUpperJointMidConeStep();
        ArmMovementComponentStep upperJointBackToIntake = mStepFactory.UpperJointStepsFactory.CreateUpperIntakingStep();


        ArmMovementComponentStep lowerJointScoochBackwards = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp");
        ArmMovementComponentStep lowerJointDownToGrabLocationStep = mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStep();
        ArmMovementComponentStep lowerJointScoochBackwards2 = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp2");    
        ArmMovementComponentStep lowerJointUpToScore = mStepFactory.LowerJointStepsFactory.CreateLowerJointMidConeStep();
        ArmMovementComponentStep lowerJointRetractToAllowUpperToFall = mStepFactory.LowerJointStepsFactory.CreateLowerJointRetractionStep();
        ArmMovementComponentStep lowerJointBackToIntake = mStepFactory.LowerJointStepsFactory.CreateLowerIntakeStep();


        
        lowerJointScoochBackwards.AddPrerequisite(new StepPrequisite(releaseClamp, 1, -1));

        upperJointUpToRotationSpot.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards, 0.01, -0.01));

        twistToGrab.AddPrerequisite(new StepPrequisite(upperJointUpToRotationSpot, 0.01, -0.01));

        upperJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        grab.AddPrerequisite(new StepPrequisite(upperJointDownToGrabLocationStep, 0.005, -0.005));
        grab.AddPrerequisite(new StepPrequisite(lowerJointDownToGrabLocationStep, 0.005, -0.005));
        pauseBeforeUpperArmGoesToScore.AddPrerequisite(new StepPrequisite(grab, 1, -1)); 

        lowerJointScoochBackwards2.AddPrerequisite(new StepPrequisite(grab, 0.01, -0.01));
        upperJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        
        

        pauseBeforeUntwist.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        twistToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUntwist, 1, -1));    
        lowerJointUpToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.03, -0.03));
        releaseToScore.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));

        lowerJointRetractToAllowUpperToFall.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        upperJointBackToIntake.AddPrerequisite(new StepPrequisite(lowerJointRetractToAllowUpperToFall, 0.01, -0.01));
        lowerJointBackToIntake.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));

        //uncomment when ready to test clamp
        closeClamp.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));
        closeClamp.AddPrerequisite(new StepPrequisite(lowerJointBackToIntake, 0.01, -0.01));

        StepSequence upperJointStepSequence = new StepSequence("UpperArm", Arrays.asList(upperJointUpToRotationSpot, upperJointDownToGrabLocationStep, upperJointUpToScore, upperJointBackToIntake));
        StepSequence lowerJointStepSequence = new StepSequence("LowerArm", Arrays.asList(lowerJointScoochBackwards, lowerJointDownToGrabLocationStep, lowerJointScoochBackwards2, lowerJointUpToScore, lowerJointRetractToAllowUpperToFall, lowerJointBackToIntake));
        StepSequence twistStepSequence = new StepSequence("Wrist", Arrays.asList(twistToGrab, twistToScore));
        StepSequence clawSequence = new StepSequence("Claw", Arrays.asList(initialOpen, grab, releaseToScore));
        StepSequence pauseSequence = new StepSequence("Pauses", Arrays.asList(pauseBeforeUpperArmGoesToScore, pauseBeforeUntwist));
        StepSequence clampSequence  = new StepSequence("Clamp", Arrays.asList(releaseClamp, closeClamp));

            //aDD CLAMPsEWQUENCE  to this list when ready to test
        Routine returnValue = new Routine(Arrays.asList(upperJointStepSequence, lowerJointStepSequence, twistStepSequence, clawSequence, pauseSequence, clampSequence));
        return returnValue;
  
    }
  

    
    public Routine CreateTopConeRoutine()
    {
      
        ClampStep releaseClamp = mStepFactory.ClampStepsFactory.CreateReleaseClampStep("InitalClampRelease");
        ClampStep closeClamp = mStepFactory.ClampStepsFactory.CreateOpenClampStep("FinalClampRelease");

        TimeDelayStep pauseBeforeUpperArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUpperArmGoesToScoreStep("pauseBeforeUpperArmGoesToScore");
        TimeDelayStep pauseBeforeUntwist = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUntwistStep("PauseBeforeUntwist");
        

        ClawStep initialOpen = mStepFactory.ClawStepsFactory.CreateOpenClawStep("InitialOpen");
        ClawStep grab = mStepFactory.ClawStepsFactory.CreateCloseClawStep("Grab");
        ClawStep releaseToScore = mStepFactory.ClawStepsFactory.CreateOpenClawStep("ReleaseToScore");


        TwistStep twistToGrab = mStepFactory.TwistStepsFactory.CreateTwistToGrab();
        TwistStep twistToScore = mStepFactory.TwistStepsFactory.CreateTwistToScore();

        ArmMovementComponentStep upperJointUpToRotationSpot = mStepFactory.UpperJointStepsFactory.CreateUpperToRotationSpotStep();
        ArmMovementComponentStep upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStep();
        ArmMovementComponentStep upperJointUpToScore = mStepFactory.UpperJointStepsFactory.CreateUpperJointTopConeStep();
        ArmMovementComponentStep upperJointBackToIntake = mStepFactory.UpperJointStepsFactory.CreateUpperIntakingStep();


        ArmMovementComponentStep lowerJointScoochBackwards = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp");
        ArmMovementComponentStep lowerJointDownToGrabLocationStep = mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStep();
        ArmMovementComponentStep lowerJointScoochBackwards2 = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp2");    
        ArmMovementComponentStep lowerJointUpToScore = mStepFactory.LowerJointStepsFactory.CreateLowerJointTopConeStep();
        ArmMovementComponentStep lowerJointRetractToAllowUpperToFall = mStepFactory.LowerJointStepsFactory.CreateLowerJointRetractionStep();
        ArmMovementComponentStep lowerJointBackToIntake = mStepFactory.LowerJointStepsFactory.CreateLowerIntakeStep();


        
        lowerJointScoochBackwards.AddPrerequisite(new StepPrequisite(releaseClamp, 1, -1));

        upperJointUpToRotationSpot.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards, 0.01, -0.01));

        twistToGrab.AddPrerequisite(new StepPrequisite(upperJointUpToRotationSpot, 0.01, -0.01));

        upperJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        grab.AddPrerequisite(new StepPrequisite(upperJointDownToGrabLocationStep, 0.005, -0.005));
        grab.AddPrerequisite(new StepPrequisite(lowerJointDownToGrabLocationStep, 0.005, -0.005));
        pauseBeforeUpperArmGoesToScore.AddPrerequisite(new StepPrequisite(grab, 1, -1)); 

        lowerJointScoochBackwards2.AddPrerequisite(new StepPrequisite(grab, 0.01, -0.01));
        upperJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        
        

        pauseBeforeUntwist.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        twistToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUntwist, 1, -1));    
        lowerJointUpToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.03, -0.03));
        releaseToScore.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));

        lowerJointRetractToAllowUpperToFall.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        upperJointBackToIntake.AddPrerequisite(new StepPrequisite(lowerJointRetractToAllowUpperToFall, 0.01, -0.01));
        lowerJointBackToIntake.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));

        //uncomment when ready to test clamp
        closeClamp.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));
        closeClamp.AddPrerequisite(new StepPrequisite(lowerJointBackToIntake, 0.01, -0.01));

        StepSequence upperJointStepSequence = new StepSequence("UpperArm", Arrays.asList(upperJointUpToRotationSpot, upperJointDownToGrabLocationStep, upperJointUpToScore, upperJointBackToIntake));
        StepSequence lowerJointStepSequence = new StepSequence("LowerArm", Arrays.asList(lowerJointScoochBackwards, lowerJointDownToGrabLocationStep, lowerJointScoochBackwards2, lowerJointUpToScore, lowerJointRetractToAllowUpperToFall, lowerJointBackToIntake));
        StepSequence twistStepSequence = new StepSequence("Wrist", Arrays.asList(twistToGrab, twistToScore));
        StepSequence clawSequence = new StepSequence("Claw", Arrays.asList(initialOpen, grab, releaseToScore));
        StepSequence pauseSequence = new StepSequence("Pauses", Arrays.asList(pauseBeforeUpperArmGoesToScore, pauseBeforeUntwist));
        StepSequence clampSequence  = new StepSequence("Clamp", Arrays.asList(releaseClamp, closeClamp));

            //aDD CLAMPsEWQUENCE  to this list when ready to test
        Routine returnValue = new Routine(Arrays.asList(upperJointStepSequence, lowerJointStepSequence, twistStepSequence, clawSequence, pauseSequence, clampSequence));
        return returnValue;
  
    }

    
}
