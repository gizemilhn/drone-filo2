import heapq
from typing import List, Tuple, Dict, Set
import numpy as np
from datetime import datetime
from drone import Drone
from delivery import Delivery
from zone import NoFlyZone

class AStarRouter:
    def __init__(self, grid_size: Tuple[int, int], resolution: float = 1.0):
        self.grid_size = grid_size
        self.resolution = resolution
        self.grid = np.zeros(grid_size)
    
    def _heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
    
    def _get_neighbors(self, pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Get valid neighboring positions."""
        x, y = pos
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            new_x, new_y = x + dx * self.resolution, y + dy * self.resolution
            if 0 <= new_x < self.grid_size[0] and 0 <= new_y < self.grid_size[1]:
                neighbors.append((new_x, new_y))
        return neighbors
    
    def _calculate_zone_penalty(self, pos: Tuple[float, float], 
                              zones: List[NoFlyZone], 
                              current_time: datetime) -> float:
        """Calculate total penalty from all no-fly zones."""
        return sum(zone.get_penalty(pos, current_time) for zone in zones)
    
    def find_path(self, 
                 start: Tuple[float, float],
                 goal: Tuple[float, float],
                 drone: Drone,
                 zones: List[NoFlyZone],
                 current_time: datetime) -> List[Tuple[float, float]]:
        """Find optimal path using A* algorithm."""
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            current = heapq.heappop(frontier)[1]
            
            if self._heuristic(current, goal) < self.resolution:
                break
            
            for next_pos in self._get_neighbors(current):
                # Calculate movement cost
                movement_cost = self._heuristic(current, next_pos)
                
                # Add penalties for no-fly zones
                zone_penalty = self._calculate_zone_penalty(next_pos, zones, current_time)
                if zone_penalty == float('inf'):
                    continue  # Skip if position is in no-fly zone
                
                new_cost = cost_so_far[current] + movement_cost + zone_penalty
                
                # Check if drone has enough battery
                if not drone.has_sufficient_battery(new_cost):
                    continue
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self._heuristic(next_pos, goal)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        
        return path if path[0] == start else []

class RouteOptimizer:
    def __init__(self, router: AStarRouter):
        self.router = router
    
    def optimize_delivery_sequence(self,
                                 drone: Drone,
                                 deliveries: List[Delivery],
                                 zones: List[NoFlyZone],
                                 current_time: datetime) -> List[Delivery]:
        """Optimize delivery sequence for a single drone."""
        # Sort deliveries by priority and time window
        sorted_deliveries = sorted(
            deliveries,
            key=lambda d: (d.priority, d.time_window_start)
        )
        
        optimized_sequence = []
        current_pos = drone.current_position
        remaining_battery = drone.current_battery
        
        for delivery in sorted_deliveries:
            if not delivery.is_within_time_window(current_time):
                continue
            
            # Find path to delivery point
            path = self.router.find_path(
                current_pos,
                delivery.position,
                drone,
                zones,
                current_time
            )
            
            if not path:
                continue
            
            # Calculate total distance
            total_distance = sum(
                self.router._heuristic(path[i], path[i+1])
                for i in range(len(path)-1)
            )
            
            # Check if drone can make the delivery
            if (drone.can_carry(delivery.weight) and
                drone.has_sufficient_battery(total_distance)):
                optimized_sequence.append(delivery)
                current_pos = delivery.position
                remaining_battery -= total_distance
        
        return optimized_sequence 