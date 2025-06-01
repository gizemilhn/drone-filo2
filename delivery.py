from dataclasses import dataclass
from typing import Tuple, Optional
from datetime import datetime, timedelta

@dataclass
class Delivery:
    id: str
    position: Tuple[float, float]
    weight: float
    priority: int
    time_window_start: datetime
    time_window_end: datetime
    assigned_drone: Optional[str] = None
    status: str = "pending"  # pending, in_progress, completed, failed
    actual_delivery_time: Optional[datetime] = None
    
    def is_within_time_window(self, current_time: datetime) -> bool:
        """Check if current time is within delivery time window."""
        return self.time_window_start <= current_time <= self.time_window_end
    
    def is_late(self, current_time: datetime) -> bool:
        """Check if delivery is late."""
        return current_time > self.time_window_end
    
    def assign_to_drone(self, drone_id: str):
        """Assign delivery to a drone."""
        self.assigned_drone = drone_id
        self.status = "in_progress"
    
    def complete_delivery(self, delivery_time: datetime):
        """Mark delivery as completed."""
        self.status = "completed"
        self.actual_delivery_time = delivery_time
    
    def fail_delivery(self):
        """Mark delivery as failed."""
        self.status = "failed"
        self.assigned_drone = None
    
    def to_dict(self) -> dict:
        """Convert delivery to dictionary for serialization."""
        return {
            'id': self.id,
            'position': self.position,
            'weight': self.weight,
            'priority': self.priority,
            'time_window_start': self.time_window_start.isoformat(),
            'time_window_end': self.time_window_end.isoformat(),
            'assigned_drone': self.assigned_drone,
            'status': self.status,
            'actual_delivery_time': self.actual_delivery_time.isoformat() if self.actual_delivery_time else None
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'Delivery':
        """Create delivery instance from dictionary."""
        return cls(
            id=data['id'],
            position=tuple(data['position']),
            weight=data['weight'],
            priority=data['priority'],
            time_window_start=datetime.fromisoformat(data['time_window_start']),
            time_window_end=datetime.fromisoformat(data['time_window_end']),
            assigned_drone=data['assigned_drone'],
            status=data['status'],
            actual_delivery_time=datetime.fromisoformat(data['actual_delivery_time']) if data['actual_delivery_time'] else None
        ) 