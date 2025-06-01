import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Tuple, Dict
import numpy as np
from datetime import datetime
from drone import Drone
from delivery import Delivery
from zone import NoFlyZone

class DeliveryVisualizer:
    def __init__(self, grid_size: Tuple[int, int]):
        self.grid_size = grid_size
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111)
        self.colors = plt.cm.rainbow(np.linspace(0, 1, 20))  # Color map for drones
    
    def plot_delivery_network(self,
                            drones: List[Drone],
                            deliveries: List[Delivery],
                            zones: List[NoFlyZone],
                            current_time: datetime):
        """Plot the complete delivery network."""
        self.ax.clear()
        
        # Set up the plot
        self.ax.set_xlim(0, self.grid_size[0])
        self.ax.set_ylim(0, self.grid_size[1])
        self.ax.set_title("Drone Delivery Network", fontsize=14, pad=20)
        self.ax.set_xlabel("X Coordinate", fontsize=12)
        self.ax.set_ylabel("Y Coordinate", fontsize=12)
        
        # Add grid
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        # Plot no-fly zones
        self._plot_no_fly_zones(zones, current_time)
        
        # Plot delivery points
        self._plot_delivery_points(deliveries)
        
        # Plot drone paths
        self._plot_drone_paths(drones)
        
        # Add legend with custom positioning
        legend = self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Adjust layout to prevent legend cutoff
        self.fig.tight_layout()
    
    def _plot_no_fly_zones(self, zones: List[NoFlyZone], current_time: datetime):
        """Plot active no-fly zones."""
        for zone in zones:
            if zone.is_active(current_time):
                # Create polygon patch
                polygon = patches.Polygon(
                    zone.polygon_coordinates,
                    closed=True,
                    fill=True,
                    alpha=0.3,
                    color='red',
                    label='No-Fly Zone'
                )
                self.ax.add_patch(polygon)
                
                # Add zone ID label
                center = np.mean(zone.polygon_coordinates, axis=0)
                self.ax.text(center[0], center[1], f'Zone {zone.id}',
                           ha='center', va='center', color='darkred',
                           fontweight='bold')
    
    def _plot_delivery_points(self, deliveries: List[Delivery]):
        """Plot delivery points with different markers based on status."""
        status_colors = {
            'pending': 'gray',
            'in_progress': 'blue',
            'completed': 'green',
            'failed': 'red'
        }
        
        for delivery in deliveries:
            color = status_colors.get(delivery.status, 'gray')
            
            # Plot delivery point
            self.ax.scatter(
                delivery.position[0],
                delivery.position[1],
                c=color,
                marker='o',
                s=100,
                label=f'Delivery {delivery.id} ({delivery.status})'
            )
            
            # Add delivery ID and weight label
            self.ax.text(
                delivery.position[0],
                delivery.position[1] + 2,
                f'D{delivery.id}\n{delivery.weight}kg',
                ha='center',
                va='bottom',
                fontsize=8
            )
    
    def _plot_drone_paths(self, drones: List[Drone]):
        """Plot drone paths with different colors."""
        for i, drone in enumerate(drones):
            color = self.colors[i % len(self.colors)]
            
            # Plot drone's current position
            self.ax.scatter(
                drone.current_position[0],
                drone.current_position[1],
                c=[color],
                marker='^',
                s=150,
                label=f'Drone {drone.id}'
            )
            
            # Add drone info label
            self.ax.text(
                drone.current_position[0],
                drone.current_position[1] + 2,
                f'Drone {drone.id}\nBattery: {drone.current_battery:.0f}\nWeight: {drone.current_weight:.1f}kg',
                ha='center',
                va='bottom',
                fontsize=8
            )
            
            # Plot drone's path
            if len(drone.route) > 1:
                path = np.array(drone.route)
                self.ax.plot(
                    path[:, 0],
                    path[:, 1],
                    c=color,
                    alpha=0.5,
                    linestyle='--',
                    linewidth=2
                )
    
    def generate_report(self,
                       drones: List[Drone],
                       deliveries: List[Delivery],
                       zones: List[NoFlyZone]) -> str:
        """Generate a text report of the delivery system status."""
        report = []
        report.append("=== Delivery System Status Report ===\n")
        
        # Drone status
        report.append("Drone Status:")
        for drone in drones:
            report.append(f"  Drone {drone.id}:")
            report.append(f"    Battery: {drone.current_battery:.2f}/{drone.battery_capacity:.2f}")
            report.append(f"    Current Weight: {drone.current_weight:.2f}/{drone.max_weight:.2f}")
            report.append(f"    Position: {drone.current_position}")
            report.append("")
        
        # Delivery status
        report.append("Delivery Status:")
        status_counts = {'pending': 0, 'in_progress': 0, 'completed': 0, 'failed': 0}
        for delivery in deliveries:
            status_counts[delivery.status] += 1
        for status, count in status_counts.items():
            report.append(f"  {status.capitalize()}: {count}")
        report.append("")
        
        # No-fly zone status
        report.append("No-Fly Zone Status:")
        for zone in zones:
            report.append(f"  Zone {zone.id}:")
            report.append(f"    Active: {zone.is_active(datetime.now())}")
            report.append(f"    Active Time: {zone.active_time_start} to {zone.active_time_end}")
            report.append("")
        
        return "\n".join(report)
    
    def save_visualization(self, filename: str):
        """Save the current visualization to a file."""
        self.fig.savefig(filename, bbox_inches='tight', dpi=300)
    
    def show(self):
        """Display the visualization."""
        plt.show() 