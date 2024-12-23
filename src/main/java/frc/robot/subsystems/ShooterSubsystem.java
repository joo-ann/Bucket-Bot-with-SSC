package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// Shooter Subsystem Code yippee
public class ShooterSubsystem extends SubsystemBase {

  // Init Stuff
  private TalonFX m_shooterIndexerMotor;
  private TalonFX m_shooterMotor;
  private TalonFXConfiguration krakenConfig;

  private static final double kShooterMomentOfInertia = 0.00032;

  private static final double kShooterGearing = 1.0;

  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

  private final LinearSystem<N1, N1, N1> m_shooterPlant = 
    LinearSystemId.createshooterSystem(
      DCMotor.getTalonFX(MotorIdConstants.k_shooterMotorID), kShooterMomentOfInertia, kShooterGearing);
  private final KalmanFilter<N1, N1, N1> m_observer =
    new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      m_shooterPlant,
      VecBuilder.fill(3.0),
      VecBuilder.fill(0.01),
      0.020
    );

  private final LinearQuadraticRegulator<N1, N1, N1> m_controller = 
    new LinearQuadraticRegulator<>(
      m_shooterPlant,
      VecBuilder.fill(8.0),
      VecBuilder.fill(12.0),
      0.020
    );

  private final LinearSystemLoop<N1, N1, N1> m_loop = 
    new LinearSystemLoop<>(m_shooterPlant, m_controller, m_observer, 12.0, 0.020);

  private final Encoder m_encoder = new Encoder(EncoderConstants.k_EncoderAChannel, k_EncoderBChannel);

  // Other setup items
  public ShooterSubsystem() {
    
    // KRAKENS
    m_shooterIndexerMotor = new TalonFX(MotorIDConstants.k_shooterIndexerMotorID);
    m_shooterMotor = new TalonFX(MotorIDConstants.k_shooterMotorID);

    // Init krakenConfig
    krakenConfig = new TalonFXConfiguration();

    // PID Stuff
    krakenConfig.Slot0.kP = MotorPIDConstants.k_intakekP;
    krakenConfig.Slot0.kI = MotorPIDConstants.k_intakekI;
    krakenConfig.Slot0.kD = MotorPIDConstants.k_intakekD;
    krakenConfig.Slot0.kS = MotorPIDConstants.k_intakekS;
    krakenConfig.Slot0.kV = MotorPIDConstants.k_intakekV;

    // Kraken Configs
    krakenConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_rampRate;
    krakenConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_closedMaxSpeed;
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;

    // Apply Configs, Inversion, Control requests
    m_shooterIndexerMotor.getConfigurator().apply(krakenConfig, 0.05);
    m_shooterMotor.getConfigurator().apply(krakenConfig, 0.05);

    m_shooterIndexerMotor.setInverted(true);
    m_shooterMotor.setInverted(false);
  }

  // Runs once per scheduler run
  public void periodic() {}
  
  // Spin up command
  public void spinUp() {
    m_loop.setNextR(VecBuilder.fill(ShooterConstants.k_shooterKrakenSpeed));
  }

  // Spin up command
  public void indexUp() {
    m_loop.setNextR(VecBuilder.fill(k_shooterIndexerKrakenSpeed));
  }

  // Shoots yippeee
  public void shoot() {
    m_shooterIndexerMotor.set(ShooterConstants.k_shooterIndexerKrakenSpeed);
  }

  // Stops motors
  public void stop() {
    m_shooterIndexerMotor.set(0);
    m_loop.setNextR(VecBuilder.fill(0.0));

    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));
    m_loop.predict(0.020);

    double nextVoltage = m_loop.getU(0);
    m_shooterMotor.setVoltage(nextVoltage);
  }

  public void stopShoot() {
    m_shooterIndexerMotor.set(0);
  }
}
