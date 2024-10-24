using System;
using System.Collections.Generic;
using System.Linq;
using Greedy.Architecture;

namespace Greedy {
    public class DijkstraData {
        // хранение предыдущего узла 
        public Point? Previous { get; set; }
        // хранение стоимости
        public int Cost { get; set; }  
    }

    public class DijkstraPathFinder {
        // константа
        private static readonly Point NoPrevious = new Point(-1, -1);  

        // поиск путей с использованием алгоритма Дейкстры
        public IEnumerable<PathWithCost> GetPathsByDijkstra(State state, Point start, 
            IEnumerable<Point> targets) {
            // хэш-множество целевых узлов
            var targetPoints = new HashSet<Point>(targets);
            // кратчайшее дерева путей
            var shortestPathTree = InitializeShortestPathTree(start);
            // хэш-множество нерассмотренных узлов
            var queueNodes = new HashSet<Point> { start };
            // хэш-множество посещённых узлов
            var visitedNodes = new HashSet<Point>();

            // пока есть не посещённые узлы
            while (queueNodes.Count > 0) {
                // получение следующего узла
                var currentNode = GetNextOpenNode(queueNodes, shortestPathTree);

                // текущий узел является целью
                if (targetPoints.Contains(currentNode)) {
                    // возврат пути с минимальной стоимостью до текущей целевого узла
                    yield return BuildPathFromShortestPathTree(shortestPathTree, currentNode);  
                }

                // обновление смежных узлов
                UpdateAdjacentNodes(currentNode, state, shortestPathTree, queueNodes, visitedNodes);  
            }
        }

        // инициализация кратчайшего дерева путей с указанной начального узла
        private Dictionary<Point, DijkstraData> InitializeShortestPathTree(Point start) {
            // словарь для кратчайшего дерева
            var shortestPathTree = new Dictionary<Point, DijkstraData>();
            // установка начального узла в дереве
            shortestPathTree[start] = new DijkstraData { Cost = 0, Previous = NoPrevious };
            // возврат кратчайшего дерева путей
            return shortestPathTree;  
        }

        // метод для получения следующей открытой вершины с наименьшей стоимостью
        private Point GetNextOpenNode(HashSet<Point> queueNodes, Dictionary<Point, 
            DijkstraData> shortestPathTree) {
            // сортировка вершин в очереди по их стоимости (затрате)
            var orderedNodes = queueNodes.OrderBy(delegate (Point node) {
                return shortestPathTree[node].Cost;
            });

            // получение первой вершины из отсортированной очереди
            var nextNode = orderedNodes.First();
            return nextNode;
        }



        // обновление смежных узлов
        private void UpdateAdjacentNodes(Point currentNode, State state, Dictionary<Point, 
            DijkstraData> shortestPathTree, HashSet<Point> queueNodes, HashSet<Point> visitedNodes) {
            // получение смежных узлов к текущему
            var adjacentNodes = GetAdjacentNodes(currentNode, state);
            // перебор смежных узлов
            foreach (var adjacentNode in adjacentNodes) {
                // обновление информации о смежных узлах
                UpdateNode(currentNode, adjacentNode, state, shortestPathTree, queueNodes, visitedNodes);  
            }
            // пометка узла как посещённого
            MarkNodeAsVisited(currentNode, queueNodes, visitedNodes);  
        }

        // обновление данных узла
        private void UpdateNode(Point fromNode, Point toNode, State state, Dictionary<Point, 
            DijkstraData> shortestPathTree, HashSet<Point> queueNodes, HashSet<Point> visitedNodes) {
            // вычисление стоимости пути до смежного узла
            var currentCost = shortestPathTree[fromNode].Cost + state.CellCost[toNode.X, toNode.Y];

            // проверка на более короткий путь до смежного узла
            if (!shortestPathTree.ContainsKey(toNode) || currentCost < shortestPathTree[toNode].Cost) {
                // обновление информации о смежном узле
                shortestPathTree[toNode] = new DijkstraData { Previous = fromNode, Cost = currentCost };  
            }

            if (!visitedNodes.Contains(toNode)) {  // проверка, не был ли посещен
                queueNodes.Add(toNode);  // добавление смежного узла в множество нерассмотренных
            }
        }

        // отметка узла как посещенного
        private void MarkNodeAsVisited(Point node, HashSet<Point> queueNodes, HashSet<Point> visitedNodes) {
            // удаление из очереди
            queueNodes.Remove(node);
            // добавление в множество посещенных узлов
            visitedNodes.Add(node);  
        }

        // построение пути с указанным кратчайшим деревом путей и конечной точкой
        public PathWithCost BuildPathFromShortestPathTree(Dictionary<Point, DijkstraData> shortestPathTree, Point end) {
            // построение пути с минимальной стоимостью до конечного узла
            var result = ReconstructPath(shortestPathTree, end);
            // получение стоимости пути
            int cost = shortestPathTree[end].Cost;
            // возврат пути с его стоимостью
            return new PathWithCost(cost, result.ToArray());  
        }

        // восстановление пути на основе кратчайшего дерева путей и конечного узла
        private List<Point> ReconstructPath(Dictionary<Point, DijkstraData> shortestPathTree, Point end) {
            // список для хранения пути
            var path = new List<Point>();
            // текущий узел как конечный узел
            Point? currentPoint = end;

            // пока не достигнута стартовый узел
            while (currentPoint != NoPrevious) {
                // добавление текущего узла в путь
                path.Add(currentPoint.Value);
                // обновление узла на предыдущий
                currentPoint = shortestPathTree[currentPoint.Value].Previous;  
            }
            // инвертирование, чтобы путь от начала к концу
            path.Reverse();
            // возврат восстановленного пути в виде списка точек
            return path; 
        }

        // получение смежных узлов относительно текущего узла
        public IEnumerable<Point> GetAdjacentNodes(Point currentNode, State state) {
            // получение всех потенциально смежных узлов
            var potentialAdjacentNodes = GetPotentialAdjacentNodes(currentNode);
            // список для действительных смежных узлов
            var validAdjacentNodes = new List<Point>();

            // перебор всех потенциально смежных узлов
            foreach (var adjacentNode in potentialAdjacentNodes) {
                // проверка, является ли действительной смежным
                if (IsNodeValid(adjacentNode, state)) {
                    // добавление действительной смежного узла в список
                    validAdjacentNodes.Add(adjacentNode);  
                }
            }

            // возврат коллекции действительных смежных узлов
            return validAdjacentNodes;  
        }

        // получение потенциальных смежных узлов
        private Point[] GetPotentialAdjacentNodes(Point currentNode) {
            // возвращение массива потенциально смежных узлов
            return new Point[]
            {
                // смежный узел справа
                new Point(currentNode.X, currentNode.Y + 1), 
                // смежный узел слева
                new Point(currentNode.X, currentNode.Y - 1),  
                // смежный узел снизу
                new Point(currentNode.X + 1, currentNode.Y), 
                // смежный узел сверху
                new Point(currentNode.X - 1, currentNode.Y)  
            };
        }

        // проверка, является ли точка действительной
        private bool IsNodeValid(Point currentNode, State state) {
            // проверка, внутри карты и не стена
            return state.InsideMap(currentNode) && !state.IsWallAt(currentNode);  
        }
    }
}