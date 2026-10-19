package frc.robot.lib.motors;

/**
 * Falhas detectadas pelos controladores com protecao ({@link SafeVelocityMotor} e
 * {@link SafePositionMotor}). Uma falha e "travada" (latched): o motor fica parado ate alguem chamar
 * {@code clearFault()} (normalmente o proximo comando do operador).
 */
public enum MotorFault {
  /** Nenhuma falha. */
  NONE("sem falha"),

  /** Leitura do encoder invalida (NaN, infinito ou valor absurdo). */
  INVALID_READING("leitura invalida do encoder (NaN/infinito/absurda)"),

  /** RPM acima do limite de seguranca (vale para sentido positivo e negativo). */
  OVERSPEED("RPM acima do limite de seguranca"),

  /** Tensao aplicada mas o encoder nao mede movimento: mecanismo travado, sem encoder ou cabo solto. */
  STALL_OR_NO_FEEDBACK("tensao aplicada sem movimento (travado, sem encoder ou cabo solto)"),

  /** O mecanismo se move no sentido OPOSTO ao comandado (motor/encoder invertido): evita disparada. */
  WRONG_DIRECTION("movimento no sentido oposto ao comandado (motor ou encoder invertido?)"),

  /** O proprio controlador do motor (SPARK/Talon) reportou uma falha. */
  CONTROLLER_FAULT("falha reportada pelo controlador do motor");

  public final String description;

  MotorFault(String description) {
    this.description = description;
  }
}
