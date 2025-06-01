from dataclasses import dataclass
from typing import List, Tuple
from datetime import datetime
from shapely.geometry import Polygon, Point

@dataclass
class NoFlyZone:
    id: str
    polygon_coordinates: List[Tuple[float, float]]
    active_time_start: datetime
    active_time_end: datetime
    
    def __post_init__(self):
        """Initialize the polygon shape."""
        self.polygon = Polygon(self.polygon_coordinates)
    
    def is_active(self, current_time: datetime) -> bool:
        """Check if the no-fly zone is active at the given time."""
        return self.active_time_start <= current_time <= self.active_time_end
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """Check if a point is within the no-fly zone."""
        return self.polygon.contains(Point(point))
    
    def intersects_line(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """Check if a line segment intersects with the no-fly zone."""
        from shapely.geometry import LineString
        line = LineString([start, end])
        return self.polygon.intersects(line)
    
    def get_penalty(self, point: Tuple[float, float], current_time: datetime) -> float:
        """Calculate penalty for being near or in the no-fly zone."""
        if not self.is_active(current_time):
            return 0.0
        
        point_obj = Point(point)
        if self.polygon.contains(point_obj):
            return float('inf')  # Infinite penalty for being inside
        
        # Calculate distance to polygon boundary
        distance = point_obj.distance(self.polygon)
        if distance < 1.0:  # Within 1 unit of the zone
            return 1000.0 / (distance + 0.1)  # Higher penalty for closer proximity
        return 0.0
    
    def to_dict(self) -> dict:
        """Convert no-fly zone to dictionary for serialization."""
        return {
            'id': self.id,
            'polygon_coordinates': self.polygon_coordinates,
            'active_time_start': self.active_time_start.isoformat(),
            'active_time_end': self.active_time_end.isoformat()
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'NoFlyZone':
        """Create no-fly zone instance from dictionary."""
        return cls(
            id=data['id'],
            polygon_coordinates=[tuple(coord) for coord in data['polygon_coordinates']],
            active_time_start=datetime.fromisoformat(data['active_time_start']),
            active_time_end=datetime.fromisoformat(data['active_time_end'])
        ) 