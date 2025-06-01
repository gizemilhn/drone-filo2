from dataclasses import dataclass
from typing import Tuple, List
import numpy as np

@dataclass
class Drone:
    id: str
    max_weight: float
    battery_capacity: float
    speed: float
    start_position: Tuple[float, float]
    current_position: Tuple[float, float] = None
    current_battery: float = None
    current_weight: float = 0.0
    route: List[Tuple[float, float]] = None
    
    def __post_init__(self):
        if self.current_position is None:
            self.current_position = self.start_position
        if self.current_battery is None:
            self.current_battery = self.battery_capacity
        if self.route is None:
            self.route = [self.start_position]
    
    def can_carry(self, weight: float) -> bool:
        """Check if drone can carry additional weight."""
        return self.current_weight + weight <= self.max_weight
    
    def update_position(self, new_position: Tuple[float, float], distance: float):
        """Update drone position and battery level."""
        self.current_position = new_position
        self.route.append(new_position)
        # Battery consumption based on distance and weight
        battery_consumption = (distance / self.speed) * (1 + 0.1 * self.current_weight)
        self.current_battery -= battery_consumption
    
    def has_sufficient_battery(self, required_distance: float) -> bool:
        """Check if drone has enough battery for a given distance."""
        estimated_consumption = (required_distance / self.speed) * (1 + 0.1 * self.current_weight)
        return self.current_battery >= estimated_consumption
    
    def reset(self):
        """Reset drone to initial state."""
        self.current_position = self.start_position
        self.current_battery = self.battery_capacity
        self.current_weight = 0.0
        self.route = [self.start_position]
    
    def to_dict(self) -> dict:
        """Convert drone to dictionary for serialization."""
        return {
            'id': self.id,
            'max_weight': self.max_weight,
            'battery_capacity': self.battery_capacity,
            'speed': self.speed,
            'start_position': self.start_position,
            'current_position': self.current_position,
            'current_battery': self.current_battery,
            'current_weight': self.current_weight,
            'route': self.route
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Drone':
        """Create drone instance from dictionary."""
        return cls(
            id=data['id'],
            max_weight=data['max_weight'],
            battery_capacity=data['battery_capacity'],
            speed=data['speed'],
            start_position=tuple(data['start_position']),
            current_position=tuple(data['current_position']),
            current_battery=data['current_battery'],
            current_weight=data['current_weight'],
            route=[tuple(pos) for pos in data['route']]
        ) 