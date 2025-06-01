import json
import sys
from datetime import datetime, timedelta
from typing import List, Dict, Tuple
import random
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                           QHBoxLayout, QPushButton, QLabel, QLineEdit,
                           QTextEdit, QComboBox, QMessageBox, QGroupBox,
                           QFormLayout, QSpinBox, QDoubleSpinBox, QTimeEdit)
from PyQt5.QtCore import Qt, QTimer, QTime
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from drone import Drone
from delivery import Delivery
from zone import NoFlyZone
from routing import AStarRouter, RouteOptimizer
from optimizer import CSPOptimizer, GAOptimizer
from visualizer import DeliveryVisualizer

class DeliverySystemGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Delivery System")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize components
        self.drones: List[Drone] = []
        self.deliveries: List[Delivery] = []
        self.zones: List[NoFlyZone] = []
        self.current_time = datetime.now()
        
        # Initialize visualizer first
        self.visualizer = DeliveryVisualizer((100, 100))
        
        # Setup UI
        self.setup_ui()
        
        # Initialize optimizers
        self.router = AStarRouter((100, 100))
        self.route_optimizer = RouteOptimizer(self.router)
        self.csp_optimizer = CSPOptimizer()
        self.ga_optimizer = None  # Will be initialized when needed
        
        # Setup timer for simulation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.simulation_speed = 1000  # ms
        
        # Load test data automatically
        self.load_test_data()
        
        # Run initial optimization
        self.run_optimization()
    
    def setup_ui(self):
        """Setup the main UI components."""
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # Left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Add data loading controls
        self.setup_data_loading_controls(left_layout)
        
        # Add drone controls
        self.setup_drone_controls(left_layout)
        
        # Add delivery controls
        self.setup_delivery_controls(left_layout)
        
        # Add no-fly zone controls
        self.setup_zone_controls(left_layout)
        
        # Add optimization controls
        self.setup_optimization_controls(left_layout)
        
        # Add simulation controls
        self.setup_simulation_controls(left_layout)
        
        # Add the panels to the main layout
        layout.addWidget(left_panel, 1)
        
        # Right panel for visualization
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Create matplotlib canvas
        self.canvas = FigureCanvas(self.visualizer.fig)
        right_layout.addWidget(self.canvas)
        
        # Add status display
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        right_layout.addWidget(self.status_display)
        
        layout.addWidget(right_panel, 2)
    
    def setup_data_loading_controls(self, layout):
        """Setup controls for loading test data."""
        group = QGroupBox("Data Management")
        group_layout = QVBoxLayout(group)
        
        # Load test data button
        load_test_btn = QPushButton("Load Test Data")
        load_test_btn.clicked.connect(self.load_test_data)
        group_layout.addWidget(load_test_btn)
        
        # Clear all data button
        clear_data_btn = QPushButton("Clear All Data")
        clear_data_btn.clicked.connect(self.clear_all_data)
        group_layout.addWidget(clear_data_btn)
        
        layout.addWidget(group)
    
    def setup_drone_controls(self, layout):
        """Setup controls for adding and managing drones."""
        group = QGroupBox("Add Drone")
        group_layout = QFormLayout(group)
        
        # Add drone form
        self.drone_id_input = QLineEdit()
        group_layout.addRow("Drone ID:", self.drone_id_input)
        
        self.drone_weight_input = QDoubleSpinBox()
        self.drone_weight_input.setRange(0.1, 100.0)
        self.drone_weight_input.setValue(5.0)
        group_layout.addRow("Max Weight:", self.drone_weight_input)
        
        self.drone_battery_input = QDoubleSpinBox()
        self.drone_battery_input.setRange(1000, 50000)
        self.drone_battery_input.setValue(10000)
        group_layout.addRow("Battery Capacity:", self.drone_battery_input)
        
        self.drone_speed_input = QDoubleSpinBox()
        self.drone_speed_input.setRange(1, 20)
        self.drone_speed_input.setValue(10)
        group_layout.addRow("Speed:", self.drone_speed_input)
        
        self.drone_x_input = QDoubleSpinBox()
        self.drone_x_input.setRange(0, 100)
        self.drone_x_input.setValue(50)
        group_layout.addRow("Start X:", self.drone_x_input)
        
        self.drone_y_input = QDoubleSpinBox()
        self.drone_y_input.setRange(0, 100)
        self.drone_y_input.setValue(50)
        group_layout.addRow("Start Y:", self.drone_y_input)
        
        add_drone_btn = QPushButton("Add Drone")
        add_drone_btn.clicked.connect(self.add_drone)
        group_layout.addRow("", add_drone_btn)
        
        layout.addWidget(group)
    
    def setup_delivery_controls(self, layout):
        """Setup controls for adding and managing deliveries."""
        group = QGroupBox("Add Delivery")
        group_layout = QFormLayout(group)
        
        # Add delivery form
        self.delivery_id_input = QLineEdit()
        group_layout.addRow("Delivery ID:", self.delivery_id_input)
        
        self.delivery_weight_input = QDoubleSpinBox()
        self.delivery_weight_input.setRange(0.1, 10.0)
        self.delivery_weight_input.setValue(1.0)
        group_layout.addRow("Weight:", self.delivery_weight_input)
        
        self.delivery_priority_input = QSpinBox()
        self.delivery_priority_input.setRange(1, 5)
        self.delivery_priority_input.setValue(3)
        group_layout.addRow("Priority:", self.delivery_priority_input)
        
        self.delivery_x_input = QDoubleSpinBox()
        self.delivery_x_input.setRange(0, 100)
        self.delivery_x_input.setValue(50)
        group_layout.addRow("Position X:", self.delivery_x_input)
        
        self.delivery_y_input = QDoubleSpinBox()
        self.delivery_y_input.setRange(0, 100)
        self.delivery_y_input.setValue(50)
        group_layout.addRow("Position Y:", self.delivery_y_input)
        
        self.delivery_time_start = QTimeEdit()
        self.delivery_time_start.setTime(QTime(0, 0))
        group_layout.addRow("Time Window Start:", self.delivery_time_start)
        
        self.delivery_time_end = QTimeEdit()
        self.delivery_time_end.setTime(QTime(1, 0))
        group_layout.addRow("Time Window End:", self.delivery_time_end)
        
        add_delivery_btn = QPushButton("Add Delivery")
        add_delivery_btn.clicked.connect(self.add_delivery)
        group_layout.addRow("", add_delivery_btn)
        
        layout.addWidget(group)
    
    def setup_zone_controls(self, layout):
        """Setup controls for adding and managing no-fly zones."""
        group = QGroupBox("Add No-Fly Zone")
        group_layout = QFormLayout(group)
        
        # Add zone form
        self.zone_id_input = QLineEdit()
        group_layout.addRow("Zone ID:", self.zone_id_input)
        
        self.zone_coords_input = QTextEdit()
        self.zone_coords_input.setPlaceholderText("Enter coordinates as x,y pairs (one per line)")
        self.zone_coords_input.setMaximumHeight(100)
        group_layout.addRow("Coordinates:", self.zone_coords_input)
        
        self.zone_time_start = QTimeEdit()
        self.zone_time_start.setTime(QTime(0, 0))
        group_layout.addRow("Active Time Start:", self.zone_time_start)
        
        self.zone_time_end = QTimeEdit()
        self.zone_time_end.setTime(QTime(2, 0))
        group_layout.addRow("Active Time End:", self.zone_time_end)
        
        add_zone_btn = QPushButton("Add No-Fly Zone")
        add_zone_btn.clicked.connect(self.add_zone)
        group_layout.addRow("", add_zone_btn)
        
        layout.addWidget(group)
    
    def setup_optimization_controls(self, layout):
        """Setup controls for running optimizations."""
        group = QGroupBox("Optimization")
        group_layout = QFormLayout(group)
        
        # Algorithm selection
        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItems(["A*", "CSP", "Genetic Algorithm"])
        group_layout.addRow("Algorithm:", self.algorithm_combo)
        
        # Run optimization button
        optimize_btn = QPushButton("Run Optimization")
        optimize_btn.clicked.connect(self.run_optimization)
        group_layout.addRow("", optimize_btn)
        
        layout.addWidget(group)
    
    def setup_simulation_controls(self, layout):
        """Setup controls for running the simulation."""
        group = QGroupBox("Simulation")
        group_layout = QFormLayout(group)
        
        # Start/Stop simulation button
        self.simulation_btn = QPushButton("Start Simulation")
        self.simulation_btn.clicked.connect(self.toggle_simulation)
        group_layout.addRow("", self.simulation_btn)
        
        # Speed control
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["Slow", "Normal", "Fast"])
        self.speed_combo.currentTextChanged.connect(self.change_simulation_speed)
        group_layout.addRow("Speed:", self.speed_combo)
        
        layout.addWidget(group)
    
    def load_test_data(self):
        """Load the test data from test_data.json."""
        try:
            with open('test_data.json', 'r') as f:
                data = json.load(f)
            
            # Clear existing data
            self.clear_all_data()
            
            # Load drones
            for drone_data in data['drones']:
                drone = Drone(
                    id=drone_data['id'],
                    max_weight=drone_data['max_weight'],
                    battery_capacity=drone_data['battery_capacity'],
                    speed=drone_data['speed'],
                    start_position=tuple(drone_data['start_position'])
                )
                self.drones.append(drone)
            
            # Load deliveries
            for delivery_data in data['deliveries']:
                delivery = Delivery(
                    id=delivery_data['id'],
                    position=tuple(delivery_data['position']),
                    weight=delivery_data['weight'],
                    priority=delivery_data['priority'],
                    time_window_start=datetime.fromisoformat(delivery_data['time_window_start']),
                    time_window_end=datetime.fromisoformat(delivery_data['time_window_end'])
                )
                self.deliveries.append(delivery)
            
            # Load no-fly zones
            for zone_data in data['no_fly_zones']:
                zone = NoFlyZone(
                    id=zone_data['id'],
                    polygon_coordinates=[tuple(coord) for coord in zone_data['polygon_coordinates']],
                    active_time_start=datetime.fromisoformat(zone_data['active_time_start']),
                    active_time_end=datetime.fromisoformat(zone_data['active_time_end'])
                )
                self.zones.append(zone)
            
            self.update_visualization()
            QMessageBox.information(self, "Success", "Test data loaded successfully!")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load test data: {str(e)}")
    
    def clear_all_data(self):
        """Clear all data from the system."""
        self.drones.clear()
        self.deliveries.clear()
        self.zones.clear()
        self.update_visualization()
    
    def add_drone(self):
        """Add a new drone to the system."""
        try:
            drone_id = self.drone_id_input.text()
            max_weight = self.drone_weight_input.value()
            battery = self.drone_battery_input.value()
            speed = self.drone_speed_input.value()
            start_pos = (self.drone_x_input.value(), self.drone_y_input.value())
            
            drone = Drone(drone_id, max_weight, battery, speed, start_pos)
            self.drones.append(drone)
            
            self.update_visualization()
            self.clear_drone_inputs()
            
        except ValueError as e:
            QMessageBox.warning(self, "Input Error", str(e))
    
    def add_delivery(self):
        """Add a new delivery to the system."""
        try:
            delivery_id = self.delivery_id_input.text()
            weight = self.delivery_weight_input.value()
            priority = self.delivery_priority_input.value()
            position = (self.delivery_x_input.value(), self.delivery_y_input.value())
            
            # Convert QTime to datetime
            base_date = datetime.now().date()
            time_start = datetime.combine(base_date, self.delivery_time_start.time())
            time_end = datetime.combine(base_date, self.delivery_time_end.time())
            
            delivery = Delivery(delivery_id, position, weight, priority,
                              time_start, time_end)
            self.deliveries.append(delivery)
            
            self.update_visualization()
            self.clear_delivery_inputs()
            
        except ValueError as e:
            QMessageBox.warning(self, "Input Error", str(e))
    
    def add_zone(self):
        """Add a new no-fly zone to the system."""
        try:
            zone_id = self.zone_id_input.text()
            coords_text = self.zone_coords_input.toPlainText()
            
            # Parse coordinates
            coords = []
            for line in coords_text.split('\n'):
                if line.strip():
                    x, y = map(float, line.strip().split(','))
                    coords.append((x, y))
            
            if len(coords) < 3:
                raise ValueError("Polygon must have at least 3 points")
            
            # Convert QTime to datetime
            base_date = datetime.now().date()
            time_start = datetime.combine(base_date, self.zone_time_start.time())
            time_end = datetime.combine(base_date, self.zone_time_end.time())
            
            zone = NoFlyZone(zone_id, coords, time_start, time_end)
            self.zones.append(zone)
            
            self.update_visualization()
            self.clear_zone_inputs()
            
        except ValueError as e:
            QMessageBox.warning(self, "Input Error", str(e))
    
    def clear_drone_inputs(self):
        """Clear drone input fields."""
        self.drone_id_input.clear()
        self.drone_weight_input.setValue(5.0)
        self.drone_battery_input.setValue(10000)
        self.drone_speed_input.setValue(10)
        self.drone_x_input.setValue(50)
        self.drone_y_input.setValue(50)
    
    def clear_delivery_inputs(self):
        """Clear delivery input fields."""
        self.delivery_id_input.clear()
        self.delivery_weight_input.setValue(1.0)
        self.delivery_priority_input.setValue(3)
        self.delivery_x_input.setValue(50)
        self.delivery_y_input.setValue(50)
        self.delivery_time_start.setTime(QTime(0, 0))
        self.delivery_time_end.setTime(QTime(1, 0))
    
    def clear_zone_inputs(self):
        """Clear zone input fields."""
        self.zone_id_input.clear()
        self.zone_coords_input.clear()
        self.zone_time_start.setTime(QTime(0, 0))
        self.zone_time_end.setTime(QTime(2, 0))
    
    def run_optimization(self):
        """Run the selected optimization algorithm."""
        algorithm = self.algorithm_combo.currentText()
        
        if algorithm == "A*":
            self.run_astar_optimization()
        elif algorithm == "CSP":
            self.run_csp_optimization()
        else:  # Genetic Algorithm
            self.run_ga_optimization()
    
    def run_astar_optimization(self):
        """Run A* optimization for each drone."""
        for drone in self.drones:
            # Reset drone route
            drone.reset()
            
            # Get optimized delivery sequence
            optimized_sequence = self.route_optimizer.optimize_delivery_sequence(
                drone, self.deliveries, self.zones, self.current_time
            )
            
            # Update drone assignments and routes
            current_pos = drone.current_position
            for delivery in optimized_sequence:
                # Find path to delivery point
                path = self.router.find_path(
                    current_pos,
                    delivery.position,
                    drone,
                    self.zones,
                    self.current_time
                )
                
                if path:
                    # Update drone route
                    for pos in path[1:]:  # Skip first position as it's already in route
                        drone.update_position(pos, self.router._heuristic(current_pos, pos))
                    
                    # Update current position
                    current_pos = delivery.position
                    
                    # Assign delivery to drone
                    delivery.assign_to_drone(drone.id)
        
        self.update_visualization()
    
    def run_csp_optimization(self):
        """Run CSP optimization."""
        assignment = self.csp_optimizer.optimize(self.drones, self.deliveries)
        
        if assignment:
            for delivery_id, drone_id in assignment.items():
                delivery = next(d for d in self.deliveries if d.id == delivery_id)
                delivery.assign_to_drone(drone_id)
        
        self.update_visualization()
    
    def run_ga_optimization(self):
        """Run Genetic Algorithm optimization."""
        if not self.ga_optimizer:
            self.ga_optimizer = GAOptimizer(self.drones, self.deliveries, self.zones)
        
        best_solution = self.ga_optimizer.optimize()
        
        # Update drone assignments based on best solution
        for i, drone_id in enumerate(best_solution):
            self.deliveries[i].assign_to_drone(drone_id)
        
        self.update_visualization()
    
    def toggle_simulation(self):
        """Start or stop the simulation."""
        if self.timer.isActive():
            self.timer.stop()
            self.simulation_btn.setText("Start Simulation")
        else:
            self.timer.start(self.simulation_speed)
            self.simulation_btn.setText("Stop Simulation")
    
    def change_simulation_speed(self, speed):
        """Change the simulation speed."""
        speeds = {
            "Slow": 2000,
            "Normal": 1000,
            "Fast": 500
        }
        self.simulation_speed = speeds[speed]
        if self.timer.isActive():
            self.timer.setInterval(self.simulation_speed)
    
    def update_simulation(self):
        """Update the simulation state."""
        self.current_time += timedelta(minutes=5)
        
        # Update drone positions
        for drone in self.drones:
            if len(drone.route) > 1:
                # Move drone to next position in route
                next_pos = drone.route[1]  # Get next position
                distance = self.router._heuristic(drone.current_position, next_pos)
                drone.update_position(next_pos, distance)
                drone.route.pop(0)  # Remove the position we just moved from
        
        self.update_visualization()
    
    def update_visualization(self):
        """Update the visualization and status display."""
        # Update visualization
        self.visualizer.plot_delivery_network(
            self.drones, self.deliveries, self.zones, self.current_time
        )
        
        # Update the canvas
        self.canvas.draw()
        
        # Update status display
        status_report = self.visualizer.generate_report(
            self.drones, self.deliveries, self.zones
        )
        self.status_display.setText(status_report)

def main():
    app = QApplication(sys.argv)
    window = DeliverySystemGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 