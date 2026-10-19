package frc.robot.lib.navigation;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utilitarios de alianca. Todo o codigo trabalha no referencial AZUL; estas funcoes convertem para
 * o lado vermelho quando necessario, usando o FlippingUtil do PathPlanner (uma unica definicao de
 * simetria do campo para joystick, destinos, obstaculos e trajetorias).
 */
public final class AllianceUtil {
  private AllianceUtil() {
    throw new UnsupportedOperationException("Classe utilitaria!");
  }

  /** @return true se estamos na alianca vermelha (false se azul ou ainda desconhecida). */
  public static boolean isRed() {
    return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
  }

  /** Converte um ponto do referencial azul para o da alianca atual. */
  public static Translation2d flipIfRed(Translation2d blue) {
    return isRed() ? FlippingUtil.flipFieldPosition(blue) : blue;
  }

  public static Rotation2d flipIfRed(Rotation2d blue) {
    return isRed() ? FlippingUtil.flipFieldRotation(blue) : blue;
  }

  public static Pose2d flipIfRed(Pose2d blue) {
    return isRed() ? FlippingUtil.flipFieldPose(blue) : blue;
  }
}
