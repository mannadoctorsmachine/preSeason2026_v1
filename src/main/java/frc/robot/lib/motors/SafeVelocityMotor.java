package frc.robot.lib.motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Motor controlado em RPM, com a malha fechada rodando na roboRIO (feedforward + PI em VOLTS) e
 * varios sistemas de seguranca. Funciona com qualquer {@link MotorController} (Vortex/NEO via
 * SparkMotor, Kraken via TalonFXMotor ou o motor simulado) porque so usa {@code setVoltage} e as
 * leituras de velocidade.
 *
 * <h2>Logica independente do sinal do encoder</h2>
 * Todas as comparacoes (alvo atingido, limite de RPM, travamento) usam o MODULO da velocidade
 * medida. Assim nao importa se o encoder le positivo ou negativo, e a malha fechada sempre tem
 * realimentacao negativa: o sentido de giro vem apenas de {@link Config#direction} (sinal da tensao
 * aplicada). Se o sinal medido for diferente do comandado, o codigo apenas avisa (diagnostico) e
 * continua funcionando.
 *
 * <h2>Protecoes</h2>
 * <ul>
 *   <li>O alvo e sempre limitado a {@link Config#maxRpm} e sobe com rampa (sem tranco de corrente).
 *   <li>Teto suave: acima de maxRpm + 2% a tensao e reduzida de forma proporcional ate voltar ao limite.
 *   <li>Falha OVERSPEED: acima de maxRpm + 8% o motor para e trava (vale para +RPM e -RPM).
 *   <li>Falha INVALID_READING: encoder com NaN/infinito/valor absurdo.
 *   <li>Falha STALL_OR_NO_FEEDBACK: tensao aplicada sem o encoder medir movimento (travado, encoder
 *       morto ou cabo solto). Sem isto a malha fechada empurraria tensao maxima as cegas.
 *   <li>Falha CONTROLLER_FAULT: falha reportada pelo proprio controlador do motor.
 *   <li>Desabilitado: motor parado e controle zerado.
 * </ul>
 *
 * <p>Uso: chamar {@link #periodic()} uma vez por ciclo (no periodic do subsistema) e
 * {@link #setTargetRpm(double)} sempre que quiser mudar o alvo. {@link #stop()} para na hora.
 */
public class SafeVelocityMotor {

  /** Configuracao. Todas as RPM sao no eixo de saida (depois da relacao de {@link MotorConfig}). */
  public static class Config {
    /** Teto de RPM: o alvo e SEMPRE limitado a este valor. */
    public final double maxRpm;

    /** Sentido de giro comandado: +1 ou -1 (sinal da tensao aplicada). */
    public int direction = 1;

    /** Tensao estatica (V) para vencer o atrito. */
    public double kS = 0.0;

    /** Feedforward de velocidade (V por RPM). Veja {@link SafeVelocityMotor#kVFromFreeSpeed(double)}. */
    public double kVVoltsPerRpm = 0.0;

    /** Ganho proporcional (V por RPM de erro). */
    public double kPVoltsPerRpm = 0.0;

    /**
     * Ganho integral (V por RPM.s de erro): elimina o erro que sobra quando o mecanismo real gira
     * menos do que o feedforward teorico preve (atrito, carga). So integra perto do alvo.
     */
    public double kIVoltsPerRpmSecond = 0.0;

    /**
     * O integral so atua quando o erro e menor que esta fracao do teto. Alem disso ele nao acumula
     * durante a rampa nem enquanto a saida esta saturada (anti-windup).
     */
    public double integralZoneFraction = 0.12;

    /** Maximo (V) que o termo integral pode somar. */
    public double integralMaxVolts = 1.5;

    /** Aceleracao (RPM/s, filtrada) abaixo da qual o RPM conta como "estavel" para o integral. */
    public double integralSteadyRpmPerSec = 1000.0;

    /** Tensao maxima aplicada ao motor (V). */
    public double maxVolts = 12.0;

    /** Aceleracao maxima do alvo (RPM por segundo). */
    public double rampRpmPerSec = 10000.0;

    /** Acima de maxRpm * (1 + isto) a tensao e reduzida (teto suave) ate voltar ao limite. */
    public double softOverspeedFraction = 0.02;

    /** Quanto a tensao cai (V) por RPM acima do teto suave. */
    public double softCapGainVoltsPerRpm = 0.01;

    /** Acima de maxRpm * (1 + isto) ocorre a falha OVERSPEED (motor para e trava). */
    public double hardOverspeedFraction = 0.08;

    /** Tensao (V) a partir da qual a ausencia de movimento conta como travamento/sem encoder. */
    public double stallMinVolts = 3.0;

    /** RPM abaixo do qual o motor e considerado parado. */
    public double stallMaxRpm = 100.0;

    /** Tempo (s) com tensao aplicada e motor parado ate declarar a falha. */
    public double stallTimeSeconds = 0.75;

    /** Se true, uma falha do proprio controlador (mantida por 0,25 s) tambem para o motor. */
    public boolean faultOnControllerFault = true;

    public Config(double maxRpm) {
      this.maxRpm = maxRpm;
    }

    public Config withDirection(int direction) {
      this.direction = (direction >= 0) ? 1 : -1;
      return this;
    }

    public Config withFeedforward(double kS, double kVVoltsPerRpm) {
      this.kS = kS;
      this.kVVoltsPerRpm = kVVoltsPerRpm;
      return this;
    }

    public Config withGains(double kPVoltsPerRpm, double kIVoltsPerRpmSecond) {
      this.kPVoltsPerRpm = kPVoltsPerRpm;
      this.kIVoltsPerRpmSecond = kIVoltsPerRpmSecond;
      return this;
    }

    public Config withMaxVolts(double maxVolts) {
      this.maxVolts = maxVolts;
      return this;
    }

    public Config withRamp(double rpmPerSecond) {
      this.rampRpmPerSec = rpmPerSecond;
      return this;
    }

    public Config withOverspeed(double softFraction, double hardFraction) {
      this.softOverspeedFraction = softFraction;
      this.hardOverspeedFraction = hardFraction;
      return this;
    }

    public Config withStallDetection(double minVolts, double stoppedBelowRpm, double seconds) {
      this.stallMinVolts = minVolts;
      this.stallMaxRpm = stoppedBelowRpm;
      this.stallTimeSeconds = seconds;
      return this;
    }

    public Config withControllerFaultCheck(boolean enabled) {
      this.faultOnControllerFault = enabled;
      return this;
    }
  }

  /**
   * Feedforward teorico: tensao nominal (12 V) dividida pela velocidade livre do motor no eixo de
   * saida. Ex: Vortex (6784 RPM livres) com reducao 1:1 => 12 / 6784 = 0,00177 V/RPM.
   *
   * @param freeSpeedRpmAtOutput RPM livres do motor medidas no eixo de saida (RPM do motor / reducao).
   */
  public static double kVFromFreeSpeed(double freeSpeedRpmAtOutput) {
    return 12.0 / freeSpeedRpmAtOutput;
  }

  private static final double kDtSeconds = 0.02;
  private static final double kInvalidReadingMultiplier = 3.0;
  private static final double kControllerFaultSeconds = 0.25;
  private static final double kPolarityMinRpm = 400.0;
  private static final double kPolarityWarnSeconds = 0.5;

  private final String name;
  private final MotorController motor;
  private final Config config;
  private final MotorInputs inputs = new MotorInputs();

  private double requestedRpm = 0.0;
  private double rampedRpm = 0.0;
  private double integral = 0.0;
  private double outputVolts = 0.0; // modulo da tensao aplicada no ultimo ciclo (>= 0)

  private double signedRpm = 0.0;
  private double speedRpm = 0.0;
  private double lastSpeedRpm = 0.0;
  private double filteredAccelRpmPerSec = 0.0;

  private MotorFault fault = MotorFault.NONE;
  private double stallSeconds = 0.0;
  private double controllerFaultSeconds = 0.0;
  private double polarityMismatchSeconds = 0.0;
  private boolean polarityWarned = false;

  /**
   * @param name Nome usado nos logs e mensagens (ex: "Shooter/Index").
   * @param motor Motor ja criado (MotorFactory). Use {@code gearRatio} no MotorConfig para que as RPM
   *     sejam as do eixo de saida.
   */
  public SafeVelocityMotor(String name, MotorController motor, Config config) {
    this.name = name;
    this.motor = motor;
    this.config = config;
  }

  // ---------------------------------------------------------------------------------------------
  // Ciclo
  // ---------------------------------------------------------------------------------------------

  /** Le o encoder, roda as protecoes e aplica a saida. Chamar UMA vez por ciclo. */
  public void periodic() {
    motor.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    signedRpm = inputs.velocityRotationsPerSec * 60.0;
    speedRpm = Math.abs(signedRpm);

    // Aceleracao do RPM filtrada (so usada para decidir quando o integral pode atuar)
    if (Double.isFinite(speedRpm) && Double.isFinite(lastSpeedRpm)) {
      double accel = (speedRpm - lastSpeedRpm) / kDtSeconds;
      filteredAccelRpmPerSec += 0.3 * (accel - filteredAccelRpmPerSec);
    }
    lastSpeedRpm = speedRpm;

    if (fault == MotorFault.NONE) {
      checkSafety();
    }

    if (fault != MotorFault.NONE || DriverStation.isDisabled()) {
      haltOutput();
    } else {
      runControl();
    }

    logOutputs();
  }

  private void checkSafety() {
    // 1) Leitura invalida
    if (!Double.isFinite(speedRpm) || speedRpm > config.maxRpm * kInvalidReadingMultiplier) {
      latch(MotorFault.INVALID_READING);
      return;
    }

    // 2) Overspeed: usa o modulo, entao protege nos dois sentidos de giro
    if (speedRpm > config.maxRpm * (1.0 + config.hardOverspeedFraction)) {
      latch(MotorFault.OVERSPEED);
      return;
    }

    // 3) Falha reportada pelo controlador do motor
    if (config.faultOnControllerFault && inputs.faulted) {
      controllerFaultSeconds += kDtSeconds;
      if (controllerFaultSeconds >= kControllerFaultSeconds) {
        latch(MotorFault.CONTROLLER_FAULT);
        return;
      }
    } else {
      controllerFaultSeconds = 0.0;
    }

    // 4) Tensao aplicada mas sem movimento: travado, encoder morto ou cabo solto
    if (outputVolts >= config.stallMinVolts && speedRpm < config.stallMaxRpm) {
      stallSeconds += kDtSeconds;
      if (stallSeconds >= config.stallTimeSeconds) {
        latch(MotorFault.STALL_OR_NO_FEEDBACK);
        return;
      }
    } else {
      stallSeconds = 0.0;
    }

    // 5) Diagnostico de polaridade (NAO e falha: a logica usa o modulo e continua funcionando)
    boolean drivenAndMoving = outputVolts > 0.0 && speedRpm > kPolarityMinRpm;
    if (drivenAndMoving && signedRpm * config.direction < 0.0) {
      polarityMismatchSeconds += kDtSeconds;
      if (polarityMismatchSeconds >= kPolarityWarnSeconds && !polarityWarned) {
        polarityWarned = true;
        DriverStation.reportWarning(
            "["
                + name
                + "] encoder le com sinal OPOSTO ao comando. A logica usa o modulo do RPM e segue"
                + " funcionando; confira o sentido de giro fisico do mecanismo.",
            false);
      }
    } else {
      polarityMismatchSeconds = 0.0;
    }
  }

  private void runControl() {
    double target = MathUtil.clamp(requestedRpm, 0.0, config.maxRpm);

    // Rampa de subida; a descida ate um novo alvo e imediata
    if (target > rampedRpm) {
      rampedRpm = Math.min(target, rampedRpm + config.rampRpmPerSec * kDtSeconds);
    } else {
      rampedRpm = target;
    }

    if (rampedRpm <= 0.0) {
      motor.stop();
      outputVolts = 0.0;
      integral = 0.0;
      return;
    }

    // Realimentacao no MODULO do RPM: sempre negativa, independente do sinal do encoder
    double error = rampedRpm - speedRpm;
    double base =
        config.kS + config.kVVoltsPerRpm * rampedRpm + config.kPVoltsPerRpm * error;

    if (config.kIVoltsPerRpmSecond > 0.0) {
      boolean rampFinished = rampedRpm >= target - 1e-6;
      // Perto do alvo (4%) o integral sempre atua; entre 4% e a zona so com o RPM estavel (sem estar
      // subindo/descendo rapido), para nao acumular durante a aproximacao e causar overshoot
      double absError = Math.abs(error);
      boolean smallError = absError <= config.maxRpm * 0.04;
      boolean mediumError = absError <= config.maxRpm * config.integralZoneFraction;
      boolean steady = Math.abs(filteredAccelRpmPerSec) < config.integralSteadyRpmPerSec;
      boolean nearTarget = smallError || (mediumError && steady);
      double unsaturated = base + config.kIVoltsPerRpmSecond * integral;
      boolean saturated =
          (unsaturated >= config.maxVolts && error > 0.0) || (unsaturated <= 0.0 && error < 0.0);

      if (!rampFinished) {
        integral = 0.0; // durante a rampa o integral nao acumula
      } else if (nearTarget && !saturated) {
        double integralLimit = config.integralMaxVolts / config.kIVoltsPerRpmSecond;
        // Acima do alvo o integral "desfaz" 3x mais rapido, para nao passar do RPM maximo
        double rate = (error < 0.0) ? 3.0 : 1.0;
        integral =
            MathUtil.clamp(integral + rate * error * kDtSeconds, -integralLimit, integralLimit);
      }
      // saturado ou longe do alvo: mantem o valor (anti-windup)
    }

    double volts = base + config.kIVoltsPerRpmSecond * integral;

    // Teto suave: acima de maxRpm + softOverspeedFraction a tensao e reduzida de forma proporcional
    // (sem cortar/frear de uma vez, o que causaria oscilacao). O integral tambem desfaz 3x mais
    // rapido com erro negativo.
    double softCapRpm = config.maxRpm * (1.0 + config.softOverspeedFraction);
    if (speedRpm > softCapRpm) {
      volts -= config.softCapGainVoltsPerRpm * (speedRpm - softCapRpm);
      integral *= 0.7; // esquece rapido o que o integral tinha acumulado a favor da subida
    }

    volts = MathUtil.clamp(volts, 0.0, config.maxVolts);

    outputVolts = volts;
    motor.setVoltage(config.direction * volts);
  }

  private void haltOutput() {
    motor.stop();
    outputVolts = 0.0;
    rampedRpm = 0.0;
    integral = 0.0;
    stallSeconds = 0.0;
  }

  private void latch(MotorFault newFault) {
    fault = newFault;
    haltOutput();
    DriverStation.reportError(
        "[" + name + "] FALHA: " + newFault.description + " (RPM medido = " + signedRpm + ")", false);
  }

  private void logOutputs() {
    Logger.recordOutput(name + "/SpeedRpm", speedRpm);
    Logger.recordOutput(name + "/SignedRpm", signedRpm);
    Logger.recordOutput(name + "/RequestedRpm", requestedRpm);
    Logger.recordOutput(name + "/RampedTargetRpm", rampedRpm);
    Logger.recordOutput(name + "/OutputVolts", config.direction * outputVolts);
    Logger.recordOutput(name + "/Fault", fault.name());
    Logger.recordOutput(
        name + "/EncoderSignDiffersFromCommand", polarityMismatchSeconds >= kPolarityWarnSeconds);
  }

  // ---------------------------------------------------------------------------------------------
  // Comandos
  // ---------------------------------------------------------------------------------------------

  /**
   * Define o alvo em RPM. Sempre usa o modulo (o sentido vem de {@link Config#direction}) e e
   * limitado a {@link Config#maxRpm}. O motor sobe ate o alvo com rampa.
   */
  public void setTargetRpm(double rpm) {
    requestedRpm = MathUtil.clamp(Math.abs(rpm), 0.0, config.maxRpm);
  }

  /** Para o motor imediatamente e zera o controle (nao limpa uma falha). */
  public void stop() {
    requestedRpm = 0.0;
    haltOutput();
  }

  /** Libera o motor depois de uma falha (o proximo comando do operador chama isto). */
  public void clearFault() {
    fault = MotorFault.NONE;
    stallSeconds = 0.0;
    controllerFaultSeconds = 0.0;
    polarityMismatchSeconds = 0.0;
    polarityWarned = false;
  }

  // ---------------------------------------------------------------------------------------------
  // Leituras
  // ---------------------------------------------------------------------------------------------

  /** @return RPM medido com sinal (o sinal depende de como o encoder le). */
  public double getSignedRpm() {
    return signedRpm;
  }

  /** @return RPM medido em modulo (use este nas comparacoes). */
  public double getSpeedRpm() {
    return speedRpm;
  }

  /** @return Alvo pedido (modulo, ja limitado ao teto). */
  public double getRequestedRpm() {
    return requestedRpm;
  }

  /**
   * @return true se ha um alvo ativo e o RPM (em modulo) ja atingiu {@code fraction} dele. Ex: 0,97
   *     com alvo 5000 => true a partir de 4850 RPM.
   */
  public boolean hasReached(double fraction) {
    return requestedRpm > 0.0 && speedRpm >= fraction * requestedRpm;
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
