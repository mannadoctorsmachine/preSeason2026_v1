package frc.robot.lib.motors;

/**
 * Interface unificada para controle e leitura de motores (Talon FX, Spark MAX, Spark FLEX e Simulado).
 * Todas as leituras e comandos operam nas unidades do mecanismo final (posicao em rotacoes/radianos,
 * velocidade em rotacoes por segundo/radianos por segundo) levando em conta a relacao de transmissao.
 */
public interface MotorController {

  /**
   * Define a saida percentual do motor (-1.0 a 1.0).
   * @param percent Fracao da tensao total.
   */
  void setPercentOutput(double percent);

  /**
   * Define a tensao direta a ser aplicada no motor.
   * @param volts Tensao entre -12.0V e +12.0V.
   */
  void setVoltage(double volts);

  /**
   * Define a posicao alvo para a malha fechada de controle on-board.
   * @param targetPositionRotations Posicao no eixo de saida (em rotacoes).
   */
  void setPosition(double targetPositionRotations);

  /**
   * Define a posicao alvo com feedforward arbitrario adicional.
   * @param targetPositionRotations Posicao no eixo de saida (em rotacoes).
   * @param arbitraryFeedforwardVolts Tensao adicional (em Volts) para compensar gravidade/atrito.
   */
  void setPosition(double targetPositionRotations, double arbitraryFeedforwardVolts);

  /**
   * Define a posicao alvo em radianos.
   * @param targetPositionRadians Posicao no eixo de saida (em radianos).
   */
  default void setPositionRadians(double targetPositionRadians) {
    setPosition(targetPositionRadians / (2.0 * Math.PI));
  }

  /**
   * Define a velocidade alvo para o controle em malha fechada on-board.
   * @param targetVelocityRotPerSec Velocidade de saida (em rotacoes por segundo).
   */
  void setVelocity(double targetVelocityRotPerSec);

  /**
   * Define a velocidade alvo com feedforward arbitrario adicional.
   * @param targetVelocityRotPerSec Velocidade de saida (em rotacoes por segundo).
   * @param arbitraryFeedforwardVolts Tensao adicional (em Volts) para aceleracao/atrito.
   */
  void setVelocity(double targetVelocityRotPerSec, double arbitraryFeedforwardVolts);

  /**
   * Define a velocidade alvo em radianos por segundo.
   * @param targetVelocityRadiansPerSec Velocidade de saida (em radianos por segundo).
   */
  default void setVelocityRadiansPerSec(double targetVelocityRadiansPerSec) {
    setVelocity(targetVelocityRadiansPerSec / (2.0 * Math.PI));
  }

  /**
   * Define a posicao alvo utilizando o gerador de perfil trapezoidal de movimento on-board (Motion Magic / MAXMotion).
   * @param targetPositionRotations Posicao alvo (em rotacoes de saida).
   */
  void setMotionProfilePosition(double targetPositionRotations);

  /**
   * Define a posicao alvo por perfil trapezoidal com feedforward arbitrario.
   * @param targetPositionRotations Posicao alvo (em rotacoes de saida).
   * @param arbitraryFeedforwardVolts Tensao de feedforward (em Volts).
   */
  void setMotionProfilePosition(double targetPositionRotations, double arbitraryFeedforwardVolts);

  /**
   * Para a atuacao do motor (saida 0V).
   */
  void stop();

  /**
   * Altera dinamicamente o modo neutro do motor (Brake vs Coast).
   * @param brake Se true ativa o freio eletromagnetico, se false permite roda livre.
   */
  void setBrakeMode(boolean brake);

  /**
   * Altera a orientacao de giro positivo do motor.
   * @param inverted Se true, inverte a direcao.
   */
  void setInverted(boolean inverted);

  /**
   * Reseta a leitura do encoder interno para uma posicao especifica (em rotacoes de saida).
   * @param positionRotations Nova posicao zero/referencia (em rotacoes de saida).
   */
  void setEncoderPosition(double positionRotations);

  /**
   * Reseta a leitura do encoder interno para uma posicao especifica (em radianos de saida).
   * @param positionRadians Nova posicao zero/referencia (em radianos de saida).
   */
  default void setEncoderPositionRadians(double positionRadians) {
    setEncoderPosition(positionRadians / (2.0 * Math.PI));
  }

  /**
   * Retorna a posicao medida no eixo de saida do mecanismo.
   * @return Posicao em rotacoes.
   */
  double getPosition();

  /**
   * Retorna a posicao medida no eixo de saida do mecanismo em radianos.
   * @return Posicao em radianos.
   */
  default double getPositionRadians() {
    return getPosition() * 2.0 * Math.PI;
  }

  /**
   * Retorna a velocidade angular medida no eixo de saida do mecanismo.
   * @return Velocidade em rotacoes por segundo (RPS).
   */
  double getVelocity();

  /**
   * Retorna a velocidade angular medida no eixo de saida do mecanismo em radianos por segundo.
   * @return Velocidade em rad/s.
   */
  default double getVelocityRadiansPerSec() {
    return getVelocity() * 2.0 * Math.PI;
  }

  /**
   * Retorna a tensao instantanea aplicada aos enrolamentos do motor.
   * @return Tensao em Volts.
   */
  double getAppliedVoltage();

  /**
   * Retorna a corrente que passa pelos enrolamentos do estator.
   * @return Corrente em Amperes.
   */
  double getStatorCurrent();

  /**
   * Retorna a corrente demandada da bateria / PDH.
   * @return Corrente em Amperes.
   */
  double getSupplyCurrent();

  /**
   * Retorna a temperatura interna do dispositivo.
   * @return Temperatura em Graus Celsius.
   */
  double getTemperature();

  /**
   * Atualiza a estrutura de telemetria para logging e determinismo de simulacao/replay.
   * @param inputs Objeto de telemetria a ser preenchido.
   */
  void updateInputs(MotorInputs inputs);

  /**
   * Aplica uma nova configuracao completa ao controlador (limites, PID, rampas, inversao).
   * @param config Objeto de configuracao.
   */
  void applyConfiguration(MotorConfig config);

  /**
   * Retorna a configuracao atual do motor.
   * @return Objeto MotorConfig.
   */
  MotorConfig getConfiguration();

  /**
   * Configura este motor para seguir outro motor mestre.
   * @param master Motor mestre.
   * @param opposeMasterDirection Se true, gira no sentido oposto ao mestre.
   */
  void follow(MotorController master, boolean opposeMasterDirection);

  /**
   * Retorna o objeto nativo do fabricante (ex: TalonFX, SparkMax) caso acesso a baixo nivel seja necessario.
   * @return Objeto nativo ou null.
   */
  Object getNativeMotor();
}
