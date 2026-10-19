package frc.robot.lib.navigation;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Algoritmo de Liang-Barsky: recorte de um segmento de reta contra um retangulo alinhado aos eixos.
 * E usado para saber (em O(1)) se o trajeto reto entre dois pontos atravessa um obstaculo.
 */
public final class LiangBarsky {
  /** Tolerancia que "encolhe" o retangulo para que tocar a borda ou o canto nao conte como colisao. */
  private static final double kEdgeEpsilon = 1e-6;

  private LiangBarsky() {
    throw new UnsupportedOperationException("Classe utilitaria!");
  }

  /**
   * Recorta o segmento P0->P1 contra o retangulo.
   * @return {tEntrada, tSaida} (parametros em [0,1] ao longo do segmento) da parte que esta dentro do
   *     retangulo, ou null se o segmento esta totalmente fora.
   */
  public static double[] clip(
      double x0, double y0, double x1, double y1,
      double xMin, double yMin, double xMax, double yMax) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double[] p = {-dx, dx, -dy, dy};
    double[] q = {x0 - xMin, xMax - x0, y0 - yMin, yMax - y0};

    double t0 = 0.0;
    double t1 = 1.0;
    for (int i = 0; i < 4; i++) {
      if (p[i] == 0.0) {
        // Segmento paralelo a esta borda: se estiver fora dela, nao ha intersecao
        if (q[i] < 0.0) {
          return null;
        }
      } else {
        double r = q[i] / p[i];
        if (p[i] < 0.0) {
          if (r > t1) {
            return null;
          }
          t0 = Math.max(t0, r);
        } else {
          if (r < t0) {
            return null;
          }
          t1 = Math.min(t1, r);
        }
      }
    }
    return new double[] {t0, t1};
  }

  /**
   * Informa se o segmento A->B atravessa o INTERIOR do retangulo. Tocar a borda, o canto ou
   * deslizar sobre uma aresta nao e considerado colisao.
   */
  public static boolean intersectsInterior(Translation2d a, Translation2d b, RectangleObstacle rect) {
    double xMin = rect.minX + kEdgeEpsilon;
    double xMax = rect.maxX - kEdgeEpsilon;
    double yMin = rect.minY + kEdgeEpsilon;
    double yMax = rect.maxY - kEdgeEpsilon;
    if (xMin >= xMax || yMin >= yMax) {
      return false;
    }
    double[] t = clip(a.getX(), a.getY(), b.getX(), b.getY(), xMin, yMin, xMax, yMax);
    return t != null && t[1] - t[0] > 1e-9;
  }
}
