from typing import List, Dict, Tuple, Set
import random
from datetime import datetime
import numpy as np
from deap import base, creator, tools, algorithms
from drone import Drone
from delivery import Delivery
from zone import NoFlyZone

class CSPOptimizer:
    def __init__(self):
        self.domains = {}  # Variable domains
        self.constraints = []  # List of constraint functions
    
    def add_constraint(self, constraint_func):
        """Add a constraint function to the CSP."""
        self.constraints.append(constraint_func)
    
    def is_consistent(self, assignment: Dict[str, str]) -> bool:
        """Check if the current assignment satisfies all constraints."""
        return all(constraint(assignment) for constraint in self.constraints)
    
    def optimize(self, drones: List[Drone], deliveries: List[Delivery]) -> Dict[str, str]:
        """Solve the CSP using backtracking search."""
        # Initialize domains
        self.domains = {
            delivery.id: [drone.id for drone in drones]
            for delivery in deliveries
        }
        
        # Add basic constraints
        self.add_constraint(self._weight_constraint)
        self.add_constraint(self._time_window_constraint)
        self.add_constraint(self._battery_constraint)
        
        return self._backtracking_search({})
    
    def _backtracking_search(self, assignment: Dict[str, str]) -> Dict[str, str]:
        """Recursive backtracking search implementation."""
        if len(assignment) == len(self.domains):
            return assignment
        
        # Select unassigned variable
        unassigned = [var for var in self.domains if var not in assignment][0]
        
        for value in self.domains[unassigned]:
            assignment[unassigned] = value
            if self.is_consistent(assignment):
                result = self._backtracking_search(assignment)
                if result is not None:
                    return result
            del assignment[unassigned]
        
        return None
    
    def _weight_constraint(self, assignment: Dict[str, str]) -> bool:
        """Check if weight constraints are satisfied."""
        drone_weights = {}
        for delivery_id, drone_id in assignment.items():
            delivery = next(d for d in self.deliveries if d.id == delivery_id)
            drone = next(d for d in self.drones if d.id == drone_id)
            
            if drone_id not in drone_weights:
                drone_weights[drone_id] = 0
            drone_weights[drone_id] += delivery.weight
            
            if drone_weights[drone_id] > drone.max_weight:
                return False
        return True
    
    def _time_window_constraint(self, assignment: Dict[str, str]) -> bool:
        """Check if time window constraints are satisfied."""
        # Implementation depends on specific time window requirements
        return True
    
    def _battery_constraint(self, assignment: Dict[str, str]) -> bool:
        """Check if battery constraints are satisfied."""
        # Implementation depends on specific battery requirements
        return True

class GAOptimizer:
    def __init__(self, drones: List[Drone], deliveries: List[Delivery], zones: List[NoFlyZone]):
        self.drones = drones
        self.deliveries = deliveries
        self.zones = zones
        
        # Create fitness and individual classes
        creator.create("FitnessMax", base.Fitness, weights=(1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMax)
        
        self.toolbox = base.Toolbox()
        self._setup_genetic_algorithm()
    
    def _setup_genetic_algorithm(self):
        """Setup genetic algorithm components."""
        # Attribute generator
        self.toolbox.register("attr_drone", random.choice, [d.id for d in self.drones])
        
        # Structure initializers
        self.toolbox.register("individual", tools.initRepeat, creator.Individual,
                            self.toolbox.attr_drone, n=len(self.deliveries))
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)
        
        # Genetic operators
        self.toolbox.register("evaluate", self._evaluate_fitness)
        self.toolbox.register("mate", tools.cxTwoPoint)
        self.toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.1)
        self.toolbox.register("select", tools.selTournament, tournsize=3)
    
    def _evaluate_fitness(self, individual: List[str]) -> Tuple[float]:
        """Evaluate the fitness of an individual."""
        # Initialize metrics
        completed_deliveries = 0
        energy_usage = 0
        constraint_violations = 0
        
        # Track drone states
        drone_states = {drone.id: {
            'current_pos': drone.start_position,
            'battery': drone.battery_capacity,
            'weight': 0
        } for drone in self.drones}
        
        # Evaluate each delivery assignment
        for delivery_idx, drone_id in enumerate(individual):
            delivery = self.deliveries[delivery_idx]
            drone_state = drone_states[drone_id]
            
            # Check weight constraint
            if drone_state['weight'] + delivery.weight > self.drones[delivery_idx].max_weight:
                constraint_violations += 1
                continue
            
            # Calculate energy required
            distance = np.sqrt(
                (delivery.position[0] - drone_state['current_pos'][0])**2 +
                (delivery.position[1] - drone_state['current_pos'][1])**2
            )
            energy_required = distance * (1 + 0.1 * drone_state['weight'])
            
            # Check battery constraint
            if energy_required > drone_state['battery']:
                constraint_violations += 1
                continue
            
            # Check no-fly zones
            if any(zone.contains_point(delivery.position) for zone in self.zones):
                constraint_violations += 1
                continue
            
            # Update drone state
            drone_state['current_pos'] = delivery.position
            drone_state['battery'] -= energy_required
            drone_state['weight'] += delivery.weight
            energy_usage += energy_required
            completed_deliveries += 1
        
        # Calculate fitness
        weight_factor = 100.0
        penalty_factor = 0.1
        heavy_penalty = 1000.0
        
        fitness = (completed_deliveries * weight_factor) - \
                 (energy_usage * penalty_factor) - \
                 (constraint_violations * heavy_penalty)
        
        return (fitness,)
    
    def optimize(self, population_size: int = 100, generations: int = 50) -> List[str]:
        """Run the genetic algorithm optimization."""
        pop = self.toolbox.population(n=population_size)
        
        # Run the algorithm
        result, logbook = algorithms.eaSimple(pop, self.toolbox,
                                            cxpb=0.7, mutpb=0.2,
                                            ngen=generations,
                                            verbose=True)
        
        # Get the best individual
        best_individual = tools.selBest(result, k=1)[0]
        return best_individual 