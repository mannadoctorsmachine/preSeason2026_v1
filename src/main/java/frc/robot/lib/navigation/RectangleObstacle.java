package frc.robot.lib.navigation;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Area retangular restrita alinhada aos eixos do campo (em metros, referencial do campo).
 * Representa estruturas, rampas, outros robos parados ou qualquer zona onde o robo nao deve entrar.
 */
public final class RectangleObstacle {
  public final double minX;
  public final double minY;
  public final double maxX;
  public final double maxY;

  /** Cria um obstaculo a partir de dois cantos opostos (em qualquer ordem). */
  public RectangleObstacle(double x1, double y1, double x2, double y2) {
    this.minX = Math.min(x1, x2);
    this.maxX = Math.max(x1, x2);
    this.minY = Math.min(y1, y2);
    this.maxY = Math.max(y1, y2);
  }

  public RectangleObstacle(Translation2d cornerA, Translation2d cornerB) {
    this(cornerA.getX(), cornerA.getY(), cornerB.getX(), cornerB.getY());
  }

  /** Cria um obstaculo a partir do centro e das dimensoes. */
  public static RectangleObstacle fromCenter(double centerX, double centerY, double width, double height) {
    return new RectangleObstacle(
        centerX - width / 2.0, centerY - height / 2.0, centerX + width / 2.0, centerY + height / 2.0);
  }

  /** @return Novo retangulo expandido em todas as direcoes por {@code margin} metros. */
  public RectangleObstacle inflated(double margin) {
    return new RectangleObstacle(minX - margin, minY - margin, maxX + margin, maxY + margin);
  }

  /** @return true se o ponto esta estritamente dentro do retangulo (a borda nao conta). */
  public boolean containsStrict(Translation2d p) {
    return p.getX() > minX && p.getX() < maxX && p.getY() > minY && p.getY() < maxY;
  }

  /** @return Os 4 cantos do retangulo. */
  public Translation2d[] corners() {
    return new Translation2d[] {
      new Translation2d(minX, minY),
      new Translation2d(maxX, minY),
      new Translation2d(maxX, maxY),
      new Translation2d(minX, maxY)
    };
  }
}
