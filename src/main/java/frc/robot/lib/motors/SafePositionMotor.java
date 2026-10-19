package frc.robot.lib.motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Motor controlado em POSICAO (rotacoes do mecanismo), com a malha fechada na roboRIO: perfil
 * trapezoidal + PID + feedforward em VOLTS, curso limitado por software e protecoes. Funciona com
 * qualquer {@link MotorController}, pois so usa {@code setVoltage} e as leituras de posicao.
 *
 * <p>Como a malha fechada esta no codigo, um encoder lendo com o sinal trocado em relacao ao motor
 * viraria realimentacao POSITIVA (o mecanismo dispararia ate o fim do curso). Por isso o controlador
 * tem um guarda: se a tensao empurra o mecanismo para o alvo mas ele se move para o lado OPOSTO, o
 * motor para na hora (falha WRONG_DIRECTION).
 *
 * <h2>Protecoes</h2>
 * <ul>
 *   <li>Alvo sempre limitado ao curso [min, max] e tensao limitada a {@link Config#maxVolts}.
 *   <li>WRONG_DIRECTION: movimento contrario ao comandado (motor ou encoder invertido).
 *   <li>STALL_OR_NO_FEEDBACK: tensao aplicada sem progresso (travado, batente, encoder morto).
 *   <li>INVALID_READING: posicao NaN ou infinita.
 *   <li>Desabilitado: motor parado e perfil sincronizado com a posicao real (sem pulo ao habilitar).
 * </ul>
 *
 * <p>Uso: {@link #periodic()} uma vez por ciclo e {@link #setGoalRotations(double)} para mover.
 */
public class SafePositionMotor {

  /** Configuracao. Posicoes em rotacoes do mecanismo (depois da relacao de {@link MotorConfig}). */
  public static class Config {
    /** Limites do curso (rotacoes). */
    public final double minRotations;

    public final double maxRotations;

    /** Ganho proporcional (V por rotacao de erro). */
    public double kP = 0.0;

    /** Ganho derivativo (V por rotacao/s). */
    public double kD = 0.0;

    /** Tensao estatica (V) para vencer o atrito quando o perfil manda mover. */
    public double kS = 0.0;

    /** Feedforward de velocidade (V por rotacao/s). Teorico: 12 / velocidade livre no eixo (rot/s). */
    public double kVVoltsPerRotPerSec = 0.0;

    /** Tensao para segurar contra a gravidade na posicao horizontal; aplicada como kG * cos(posicao). */
    public double kG = 0.0;

    /** Posicao (rotacoes) em que o braco esta horizontal (para o cosseno da gravidade). */
    public double gravityZeroRotations = 0.0;

    /** Tensao maxima aplicada (V). Limita forca e velocidade (nao bate forte no batente). */
    public double maxVolts = 6.0;

    /** Limites do perfil de movimento. */
    public double maxVelocityRotPerSec = 0.5;

    public double maxAccelerationRotPerSecSq = 2.0;

    /** Distancia ao alvo (rotacoes) para considerar "chegou". */
    public double toleranceRotations = 0.005;

    /** Tensao (V) a partir da qual o guarda passa a vigiar o movimento. */
    public double guardMinVolts = 1.5;

    /** Janela (s) para decidir que o mecanismo esta travado (sem progresso). */
    public double guardWindowSeconds = 0.35;

    /** Tempo (s) de vigilancia antes de poder declarar movimento no sentido oposto. */
    public double wrongDirectionSeconds = 0.08;

    /**
     * Ao trocar o alvo com o mecanismo JA em movimento (ex: inverter o sentido no meio do curso), o
     * guarda espera este tempo (s) para a inercia acabar antes de vigiar de novo.
     */
    public double guardGraceSeconds = 0.30;

    /** Velocidade (rotacoes/s) acima da qual o mecanismo conta como "em movimento" para a carencia. */
    public double graceVelocityRotPerSec = 0.10;

    /** Movimento oposto (rotacoes) a partir do qual e falha WRONG_DIRECTION. */
    public double wrongDirectionRotations = 0.01;

    /** Progresso minimo (rotacoes) na janela; abaixo disto e falha STALL_OR_NO_FEEDBACK. */
    public double stallProgressRotations = 0.002;

    public Config(double minRotations, double maxRotations) {
      this.minRotations = Math.min(minRotations, maxRotations);
      this.maxRotations = Math.max(minRotations, maxRotations);
    }

    public Config withGains(double kP, double kD) {
      this.kP = kP;
      this.kD = kD;
      return this;
    }

    public Config withFeedforward(double kS, double kVVoltsPerRotPerSec) {
      this.kS = kS;
      this.kVVoltsPerRotPerSec = kVVoltsPerRotPerSec;
      return this;
    }

    public Config withGravity(double kG, double horizontalRotations) {
      this.kG = kG;
      this.gravityZeroRotations = horizontalRotations;
      return this;
    }

    public Config withMaxVolts(double maxVolts) {
      this.maxVolts = maxVolts;
      return this;
    }

    public Config withProfile(double maxVelocityRotPerSec, double maxAccelerationRotPerSecSq) {
      this.maxVelocityRotPerSec = maxVelocityRotPerSec;
      this.maxAccelerationRotPerSecSq = maxAccelerationRotPerSecSq;
      return this;
    }

    public Config withTolerance(double rotations) {
      this.toleranceRotations = rotations;
      return this;
    }
  }

  /**
   * Feedforward teorico de velocidade: 12 V dividido pela velocidade livre do motor no eixo de saida.
   *
   * @param freeSpeedRotPerSecAtOutput velocidade livre em rotacoes/s no mecanismo (motor / reducao).
   */
  public static double kVFromFreeSpeed(double freeSpeedRotPerSecAtOutput) {
    return 12.0 / freeSpeedRotPerSecAtOutput;
  }

  private static final double kDtSeconds = 0.02;

  private final String name;
  private final MotorController motor;
  private final Config config;
  private final MotorInputs inputs = new MotorInputs();
  private final ProfiledPIDController controller;

  private double goalRotations = 0.0;
  private double appliedVolts = 0.0;
  private MotorFault fault = MotorFault.NONE;

  private int guardSign = 0;
  private double guardElapsedSeconds = 0.0;
  private double guardStartPosition = 0.0;
  private double guardGraceRemainingSeconds = 0.0;

  /**
   * @param name Nome usado nos logs e mensagens (ex: "Intake/Pivot").
   * @param motor Motor ja criado (MotorFactory), com {@code gearRatio} para que a posicao seja a do
   *     mecanismo.
   */
  public SafePositionMotor(String name, MotorController motor, Config config) {
    this.name = name;
    this.motor = motor;
    this.config = config;
    this.controller =
        new ProfiledPIDController(
            config.kP,
            0.0,
            config.kD,
            new TrapezoidProfile.Constraints(
                config.maxVelocityRotPerSec, config.maxAccelerationRotPerSecSq));
    this.controller.setTolerance(config.toleranceRotations);
  }

  // ---------------------------------------------------------------------------------------------
  // Ciclo
  // ---------------------------------------------------------------------------------------------

  /** Le o encoder, roda as protecoes e aplica a saida. Chamar UMA vez por ciclo. */
  public void periodic() {
    motor.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    double position = inputs.positionRotations;

    if (fault == MotorFault.NONE && !Double.isFinite(position)) {
      latch(MotorFault.INVALID_READING, position);
    }

    if (fault != MotorFault.NONE || DriverStation.isDisabled()) {
      haltOutput(position);
      logOutputs(position);
      return;
    }

    double pid = controller.calculate(position);
    TrapezoidProfile.State setpoint = controller.getSetpoint();

    double gravity =
        config.kG * Math.cos(2.0 * Math.PI * (position - config.gravityZeroRotations));
    double feedforward =
        config.kS * Math.signum(setpoint.velocity)
            + config.kVVoltsPerRotPerSec * setpoint.velocity
            + gravity;
    double volts = MathUtil.clamp(pid + feedforward, -config.maxVolts, config.maxVolts);

    // Limite de curso por software: nunca empurra para FORA do curso (so permite voltar)
    if (position >= config.maxRotations && volts > 0.0) {
      volts = 0.0;
    }
    if (position <= config.minRotations && volts < 0.0) {
      volts = 0.0;
    }

    runGuard(position, volts);
    if (fault != MotorFault.NONE) {
      haltOutput(position);
      logOutputs(position);
      return;
    }

    appliedVolts = volts;
    motor.setVoltage(volts);
    logOutputs(position);
  }

  /**
   * Vigia o movimento enquanto o controlador empurra o mecanismo PARA o alvo. Ignora o caso de
   * frenagem (tensao contraria ao erro) e o mecanismo ja no alvo.
   */
  private void runGuard(double position, double volts) {
    int sign;
    if (Math.abs(volts) < config.guardMinVolts) {
      sign = 0;
    } else {
      sign = (volts > 0.0) ? 1 : -1;
    }
    int errorSign = (int) Math.signum(goalRotations - position);

    // Carencia depois de inverter o alvo com o mecanismo em movimento (deixa a inercia acabar)
    if (guardGraceRemainingSeconds > 0.0) {
      guardGraceRemainingSeconds -= kDtSeconds;
      restartGuard(position, 0);
      return;
    }

    if (sign == 0 || sign != errorSign || atGoal()) {
      restartGuard(position, 0);
      return;
    }
    if (sign != guardSign) {
      restartGuard(position, sign);
      return;
    }

    guardElapsedSeconds += kDtSeconds;
    double progress = (position - guardStartPosition) * sign; // > 0: andou no sentido da tensao

    if (guardElapsedSeconds >= config.wrongDirectionSeconds
        && progress < -config.wrongDirectionRotations) {
      latch(MotorFault.WRONG_DIRECTION, position);
      return;
    }
    if (guardElapsedSeconds >= config.guardWindowSeconds) {
      if (progress < config.stallProgressRotations) {
        latch(MotorFault.STALL_OR_NO_FEEDBACK, position);
      } else {
        restartGuard(position, sign);
      }
    }
  }

  private void restartGuard(double position, int sign) {
    guardSign = sign;
    guardElapsedSeconds = 0.0;
    guardStartPosition = position;
  }

  private void haltOutput(double position) {
    motor.stop();
    appliedVolts = 0.0;
    restartGuard(position, 0);
    if (Double.isFinite(position)) {
      controller.reset(position); // perfil acompanha a posicao real: sem pulo ao voltar a operar
    }
  }

  private void latch(MotorFault newFault, double position) {
    fault = newFault;
    DriverStation.reportError(
        "["
            + name
            + "] FALHA: "
            + newFault.description
            + " (posicao = "
            + position
            + " rot, alvo = "
            + goalRotations
            + " rot)",
        false);
  }

  private void logOutputs(double position) {
    Logger.recordOutput(name + "/PositionRotations", position);
    Logger.recordOutput(name + "/GoalRotations", goalRotations);
    Logger.recordOutput(name + "/SetpointRotations", controller.getSetpoint().position);
    Logger.recordOutput(name + "/OutputVolts", appliedVolts);
    Logger.recordOutput(name + "/AtGoal", atGoal());
    Logger.recordOutput(name + "/Fault", fault.name());
  }

  // ---------------------------------------------------------------------------------------------
  // Comandos
  // ---------------------------------------------------------------------------------------------

  /** Define o alvo (rotacoes), sempre limitado ao curso. O movimento segue o perfil trapezoidal. */
  public void setGoalRotations(double rotations) {
    double newGoal = MathUtil.clamp(rotations, config.minRotations, config.maxRotations);
    if (newGoal != goalRotations) {
      boolean movingNow =
          Math.abs(inputs.velocityRotationsPerSec) > config.graceVelocityRotPerSec;
      guardGraceRemainingSeconds = movingNow ? config.guardGraceSeconds : 0.0;
      restartGuard(inputs.positionRotations, 0);
    }
    goalRotations = newGoal;
    controller.setGoal(goalRotations);
  }

  /**
   * Informa a posicao real conhecida (ex: ao ligar o robo com o mecanismo no batente). Ajusta o
   * encoder do motor e o perfil, e adota essa posicao como alvo.
   */
  public void resetPosition(double rotations) {
    motor.setEncoderPosition(rotations);
    inputs.positionRotations = rotations;
    controller.reset(rotations);
    goalRotations = MathUtil.clamp(rotations, config.minRotations, config.maxRotations);
    controller.setGoal(goalRotations);
    guardGraceRemainingSeconds = 0.0;
    restartGuard(rotations, 0);
  }

  /** Libera o motor depois de uma falha (o proximo pedido do operador chama isto). */
  public void clearFault() {
    fault = MotorFault.NONE;
    restartGuard(inputs.positionRotations, 0);
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  public double getVelocityRotPerSec() {
    return inputs.velocityRotationsPerSec;
  }

  public double getGoalRotations() {
    return goalRotations;
  }

  /** @return true se a posicao medida esta dentro da tolerancia do alvo. */
  public boolean atGoal() {
    return Math.abs(inputs.positionRotations - goalRotations) <= config.toleranceRotations;
  }

  public boolean isFaulted() {
    return fault != MotorFault.NONE;
  }

  public MotorFault getFault() {
    return fault;
  }

  public double getCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public String getName() {
    return name;
  }
}
