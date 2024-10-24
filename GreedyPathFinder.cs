using System.Collections.Generic;
using System.Linq;
using Greedy.Architecture;

namespace Greedy {
    public class GreedyPathFinder : IPathFinder {

        // нахождения пути к цели
        public List<Point> FindPathToCompleteGoal(State state) {
            // если цель уже достигнута
            if (state.Goal == 0)
                // вернуть пустой список
                return new List<Point>();

            // хэш-множество доступных сундуков
            HashSet<Point> availableChests = new HashSet<Point>(state.Chests);

            // поиск пути к цели
            List<Point> pathToCompleteGoal = FindPathToGoal(state, state.Position, 0, availableChests);

            // вернуть найденный путь
            return pathToCompleteGoal;
        }

        // нахождение пути к цели с учетом доступных сундуков
        private List<Point> FindPathToGoal(State state, Point currentPosition, 
            int currentCost, HashSet<Point> availableChests) {
            // список для хранения пути к цели
            List<Point> pathToGoal = new List<Point>();

            // объект для поиска пути методом Дейкстры
            DijkstraPathFinder pathFinder = new DijkstraPathFinder();

            // цикл по количеству шагов
            for (int step = 0; step < state.Goal; step++) {
                // нет доступных сундуков
                if (!availableChests.Any())
                    // вернуть пустой список
                    return new List<Point>();

                // найти кратчайший путь к ближайшему сундуку
                PathWithCost shortestPathToChest = FindShortestPathToNearestChest(state, currentPosition,
                    availableChests, pathFinder);

                // путь к сундуку не найден
                if (shortestPathToChest == null)
                    // вернуть пустой список
                    return new List<Point>();

                // обновить текущее положение
                currentPosition = shortestPathToChest.End;
                // обновить текущую стоимость пути
                currentCost += shortestPathToChest.Cost;

                // стоимость пути превышает доступную энергию
                if (currentCost > state.Energy)
                    // вернуть пустой список
                    return new List<Point>();

                // убрать сундук из доступных
                availableChests.Remove(shortestPathToChest.End);
                // добавить часть путив общий путь
                pathToGoal.AddRange(shortestPathToChest.Path.Skip(1)); 
            }

            // вернуть путь к цели
            return pathToGoal; 
        }

        // нахождение кратчайшего пути к ближайшему сундуку с использованием алгоритма Дейкстры
        private PathWithCost FindShortestPathToNearestChest(State state, Point currentPosition, 
            HashSet<Point> availableChests, DijkstraPathFinder pathFinder) {
            // получаем пути с использованием Dijkstra
            var paths = pathFinder.GetPathsByDijkstra(state, currentPosition, availableChests);

            // получаем первый найденный путь
            var firstPath = paths.FirstOrDefault();

            // возвращаем первый найденный путь
            return firstPath;
        }
    }
}
