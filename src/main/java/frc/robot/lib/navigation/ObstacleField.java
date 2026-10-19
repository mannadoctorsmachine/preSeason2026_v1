package frc.robot.lib.navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Conjunto de zonas restritas do campo + planejador de rotas que contorna essas zonas.
 *
 * <p>Como funciona:
 * <ol>
 *   <li>Cada obstaculo e expandido pelo raio do robo (assim o planejador trata o robo como um ponto).
 *   <li>Se o trajeto reto start->goal nao atravessa nenhum obstaculo (Liang-Barsky), a rota e direta.
 *   <li>Caso contrario monta-se um grafo de visibilidade: nos = start, goal e os cantos dos obstaculos
 *       expandidos; arestas = segmentos livres de colisao (Liang-Barsky). Dijkstra devolve a rota mais
 *       curta, que passa rente aos cantos, com a folga do raio do robo.
 * </ol>
 * Funciona para varios obstaculos, inclusive sobrepostos.
 */
public class ObstacleField {
  /** Dimensoes padrao do campo FRC (m), usadas para descartar cantos fora do campo. */
  public static final double kDefaultFieldLength = 16.541;
  public static final double kDefaultFieldWidth = 8.069;

  private final List<RectangleObstacle> obstacles = new ArrayList<>();
  private final List<RectangleObstacle> inflated = new ArrayList<>();
  private final double robotRadiusMeters;
  private final double fieldLength;
  private final double fieldWidth;

  // Cache do grafo estatico (cantos <-> cantos); recalculado somente quando os obstaculos mudam
  private Translation2d[] cornerNodes = new Translation2d[0];
  private boolean[][] cornerEdges = new boolean[0][0];
  private boolean dirty = true;

  /**
   * @param robotRadiusMeters Raio do circulo que envolve o robo + folga de seguranca (ex: 0.45 m).
   */
  public ObstacleField(double robotRadiusMeters) {
    this(robotRadiusMeters, kDefaultFieldLength, kDefaultFieldWidth);
  }

  public ObstacleField(double robotRadiusMeters, double fieldLength, double fieldWidth) {
    this.robotRadiusMeters = robotRadiusMeters;
    this.fieldLength = fieldLength;
    this.fieldWidth = fieldWidth;
  }

  // ---------------------------------------------------------------------------------------------
  // Cadastro de obstaculos
  // ---------------------------------------------------------------------------------------------

  /** Adiciona uma zona restrita. @return this (para encadear). */
  public ObstacleField addObstacle(RectangleObstacle obstacle) {
    obstacles.add(obstacle);
    inflated.add(obstacle.inflated(robotRadiusMeters));
    dirty = true;
    return this;
  }

  /** Adiciona uma zona restrita por dois cantos opostos (metros, referencial do campo). */
  public ObstacleField addObstacle(double x1, double y1, double x2, double y2) {
    return addObstacle(new RectangleObstacle(x1, y1, x2, y2));
  }

  /** Remove todas as zonas restritas. */
  public void clear() {
    obstacles.clear();
    inflated.clear();
    dirty = true;
  }

  public List<RectangleObstacle> getObstacles() {
    return List.copyOf(obstacles);
  }

  /**
   * @return Este campo se estamos na alianca azul; senao uma copia com os obstaculos espelhados
   *     (mesma simetria do PathPlanner). Cadastre os obstaculos sempre no referencial azul.
   */
  public ObstacleField forCurrentAlliance() {
    if (!AllianceUtil.isRed()) {
      return this;
    }
    ObstacleField flipped = new ObstacleField(robotRadiusMeters, fieldLength, fieldWidth);
    for (RectangleObstacle o : obstacles) {
      Translation2d a = AllianceUtil.flipIfRed(new Translation2d(o.minX, o.minY));
      Translation2d b = AllianceUtil.flipIfRed(new Translation2d(o.maxX, o.maxY));
      flipped.addObstacle(new RectangleObstacle(a, b));
    }
    return flipped;
  }

  // ---------------------------------------------------------------------------------------------
  // Consultas
  // ---------------------------------------------------------------------------------------------

  /** @return true se o centro do robo em {@code p} estaria dentro de uma zona (com folga do robo). */
  public boolean isInsideObstacle(Translation2d p) {
    for (RectangleObstacle o : inflated) {
      if (o.containsStrict(p)) {
        return true;
      }
    }
    return false;
  }

  /** @return true se o trajeto reto a->b nao atravessa nenhuma zona (considerando o raio do robo). */
  public boolean isPathClear(Translation2d a, Translation2d b) {
    for (RectangleObstacle o : inflated) {
      if (LiangBarsky.intersectsInterior(a, b, o)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Calcula a rota mais curta de start ate goal desviando das zonas.
   *
   * @return Lista de pontos a visitar em ordem. NAO inclui start e SEMPRE termina em goal.
   *     Optional.empty() se nao existir caminho livre (ex: goal dentro de uma zona).
   */
  public Optional<List<Translation2d>> planPath(Translation2d start, Translation2d goal) {
    if (obstacles.isEmpty() || isPathClear(start, goal)) {
      return Optional.of(List.of(goal));
    }
    rebuildIfDirty();

    int n = cornerNodes.length;
    int startIdx = n;
    int goalIdx = n + 1;
    Translation2d[] nodes = Arrays.copyOf(cornerNodes, n + 2);
    nodes[startIdx] = start;
    nodes[goalIdx] = goal;

    // Dijkstra O(V^2) - o grafo e pequeno (4 nos por obstaculo)
    int total = n + 2;
    double[] dist = new double[total];
    int[] prev = new int[total];
    boolean[] done = new boolean[total];
    Arrays.fill(dist, Double.POSITIVE_INFINITY);
    Arrays.fill(prev, -1);
    dist[startIdx] = 0.0;

    for (int iter = 0; iter < total; iter++) {
      int u = -1;
      double best = Double.POSITIVE_INFINITY;
      for (int i = 0; i < total; i++) {
        if (!done[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u == -1 || u == goalIdx) {
        break;
      }
      done[u] = true;

      for (int v = 0; v < total; v++) {
        if (done[v] || v == u || nodes[v] == null) {
          continue;
        }
        boolean edge;
        if (u < n && v < n) {
          edge = cornerEdges[u][v];
        } else {
          edge = isEdgeClear(nodes[u], nodes[v], u >= n, v >= n);
        }
        if (!edge) {
          continue;
        }
        double alt = dist[u] + nodes[u].getDistance(nodes[v]);
        if (alt < dist[v]) {
          dist[v] = alt;
          prev[v] = u;
        }
      }
    }

    if (prev[goalIdx] == -1) {
      return Optional.empty();
    }

    List<Translation2d> reversed = new ArrayList<>();
    for (int at = goalIdx; at != startIdx; at = prev[at]) {
      reversed.add(nodes[at]);
    }
    List<Translation2d> path = new ArrayList<>(reversed.size());
    for (int i = reversed.size() - 1; i >= 0; i--) {
      path.add(reversed.get(i));
    }
    return Optional.of(path);
  }

  /** @return Comprimento total (m) de start passando por todos os pontos da rota. */
  public static double pathLength(Translation2d start, List<Translation2d> path) {
    double length = 0.0;
    Translation2d last = start;
    for (Translation2d p : path) {
      length += last.getDistance(p);
      last = p;
    }
    return length;
  }

  // ---------------------------------------------------------------------------------------------
  // Internos
  // ---------------------------------------------------------------------------------------------

  /**
   * Aresta com pelo menos uma ponta livre (start/goal). Se a ponta esta dentro da zona de folga de um
   * obstaculo, essa zona e testada SEM a folga, permitindo ao robo "sair" de perto do obstaculo sem
   * jamais atravessar o obstaculo real.
   */
  private boolean isEdgeClear(Translation2d a, Translation2d b, boolean aFree, boolean bFree) {
    for (int i = 0; i < inflated.size(); i++) {
      RectangleObstacle zone = inflated.get(i);
      boolean exempt =
          (aFree && zone.containsStrict(a)) || (bFree && zone.containsStrict(b));
      RectangleObstacle test = exempt ? obstacles.get(i) : zone;
      if (LiangBarsky.intersectsInterior(a, b, test)) {
        return false;
      }
    }
    return true;
  }

  private void rebuildIfDirty() {
    if (!dirty) {
      return;
    }
    List<Translation2d> valid = new ArrayList<>();
    for (RectangleObstacle zone : inflated) {
      for (Translation2d corner : zone.corners()) {
        boolean insideField =
            corner.getX() >= 0.0 && corner.getX() <= fieldLength
                && corner.getY() >= 0.0 && corner.getY() <= fieldWidth;
        if (insideField && !isInsideObstacle(corner)) {
          valid.add(corner);
        }
      }
    }
    cornerNodes = valid.toArray(new Translation2d[0]);
    int n = cornerNodes.length;
    cornerEdges = new boolean[n][n];
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        boolean clear = isPathClear(cornerNodes[i], cornerNodes[j]);
        cornerEdges[i][j] = clear;
        cornerEdges[j][i] = clear;
      }
    }
    dirty = false;
  }
}
