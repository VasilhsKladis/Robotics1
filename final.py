import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.linalg import block_diag, inv
from matplotlib.patches import Ellipse, Rectangle, Wedge
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2

# Constants for LEGO RC car
WHEELBASE = 0.5  # 0.5m wheelbase
MAX_STEER_ANGLE = np.radians(30)  # ±30 degrees steering angle
MAX_SPEED = 0.4  # m/s
MIN_TURN_RADIUS = WHEELBASE / np.tan(MAX_STEER_ANGLE)

class DataAssociation:
    """Handle data association for unknown correspondences"""
    def __init__(self, max_range=5.0, gate_threshold=9.21, new_landmark_threshold=5.0):
        self.max_range = max_range
        self.gate_threshold = gate_threshold
        self.new_landmark_threshold = new_landmark_threshold
        
    def associate(self, observations, robot_pose, landmarks, landmark_covariances, 
                  landmark_initialized, bearing_noise):
        if not observations:
            return [], []
            
        associations = []
        new_observations = []
        used_landmarks = set()
        
        obs_bearings = np.array([obs[1] for obs in observations])
        
        initialized_landmarks = [(i, pos, cov) for i, (pos, cov) in 
                                enumerate(zip(landmarks, landmark_covariances)) 
                                if landmark_initialized[i] and cov is not None]
        
        if not initialized_landmarks:
            new_observations = list(range(len(observations)))
            return associations, new_observations
        
        expected_bearings = []
        valid_landmarks = []
        
        rx, ry, rtheta = robot_pose
        
        for landmark_id, pos, cov in initialized_landmarks:
            lx, ly = pos
            dx = lx - rx
            dy = ly - ry
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > self.max_range:
                continue
                
            expected_bearing = np.arctan2(dy, dx) - rtheta
            expected_bearing = (expected_bearing + np.pi) % (2 * np.pi) - np.pi
            
            q = dx**2 + dy**2
            H_landmark = np.array([-dy/q, dx/q])
            S = H_landmark @ cov @ H_landmark.T + bearing_noise**2
            
            expected_bearings.append(expected_bearing)
            valid_landmarks.append((landmark_id, S))
        
        if not expected_bearings:
            new_observations = list(range(len(observations)))
            return associations, new_observations
        
        expected_bearings = np.array(expected_bearings)
        
        BIG_NUMBER = 1e10
        cost_matrix = np.full((len(observations), len(valid_landmarks)), BIG_NUMBER)
        
        for obs_idx, (_, obs_bearing) in enumerate(observations):
            for lm_idx, (landmark_id, S) in enumerate(valid_landmarks):
                innovation = obs_bearing - expected_bearings[lm_idx]
                innovation = (innovation + np.pi) % (2 * np.pi) - np.pi
                mahalanobis_sq = innovation**2 / S
                
                if mahalanobis_sq <= self.gate_threshold:
                    cost_matrix[obs_idx, lm_idx] = mahalanobis_sq
        
        if np.all(cost_matrix >= BIG_NUMBER):
            new_observations = list(range(len(observations)))
            return associations, new_observations
        
        try:
            row_indices, col_indices = linear_sum_assignment(cost_matrix)
            
            for obs_idx, lm_idx in zip(row_indices, col_indices):
                if cost_matrix[obs_idx, lm_idx] < BIG_NUMBER:
                    landmark_id = valid_landmarks[lm_idx][0]
                    associations.append((obs_idx, landmark_id))
                    used_landmarks.add(landmark_id)
                else:
                    new_observations.append(obs_idx)
        except Exception as e:
            print(f"Hungarian algorithm failed: {e}")
            new_observations = list(range(len(observations)))
            return associations, new_observations
        
        assigned_obs = set(assoc[0] for assoc in associations)
        for obs_idx in range(len(observations)):
            if obs_idx not in assigned_obs:
                new_observations.append(obs_idx)
        
        return associations, new_observations

class ImprovedBearingOnlyEKF_SLAM:
    def __init__(self, initial_pose, num_landmarks, motion_noise, bearing_noise, max_range=10.0):
        self.mu = np.array(initial_pose)
        self.Sigma = np.eye(3) * 1e-4  # Reduced initial uncertainty
        
        self.landmark_positions = np.zeros((num_landmarks, 2))
        self.landmark_covariances = [None] * num_landmarks
        self.landmark_initialized = [False] * num_landmarks
        self.landmark_observed = [False] * num_landmarks
        
        self.landmark_observations = [[] for _ in range(num_landmarks)]
        self.landmark_init_threshold = 5
        self.min_baseline = 1.5
        self.min_triangulation_angle = np.radians(30)
        self.max_triangulation_angle = np.radians(150)
        self.max_observation_history = 20
        
        # Reduced motion noise for better tracking
        self.motion_noise = np.diag([x * 0.5 for x in motion_noise]) ** 2  # Reduced noise
        self.bearing_noise = bearing_noise
        self.max_range = max_range
        
        self.state_size = 3 + 2 * num_landmarks
        self.mu_full = np.zeros(self.state_size)
        self.mu_full[:3] = self.mu
        self.Sigma_full = np.eye(self.state_size) * 1e-6
        self.Sigma_full[:3, :3] = self.Sigma
        
        # Enhanced parameters for better stability
        self.landmark_init_threshold = 3  # Reduced threshold for faster initialization
        self.min_baseline = 1.0  # Smaller baseline requirement
        self.min_triangulation_angle = np.radians(20)  # More flexible angle constraints
        self.max_triangulation_angle = np.radians(160)
        self.max_observation_history = 30  # More observations for better triangulation
        
        # New: Landmark quality tracking
        self.landmark_quality = [0] * num_landmarks  # Track how well each landmark is observed
        self.min_quality_for_update = 3  # Minimum quality score for updates
        self.quality_decay = 0.95  # Quality decay factor per step
        
        self.history = {
            'poses': [self.mu.copy()],
            'covariances': [self.Sigma.copy()],
            'landmarks': [self.landmark_positions.copy()]
        }
    
    def predict(self, u, dt):
        v, steer_angle = u  # Now using steering angle instead of angular velocity
        
        # Constrain steering angle to physical limits
        steer_angle = np.clip(steer_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
        
        # Current state
        x, y, theta = self.mu
        
        # Improved Ackermann steering model with better numerical stability
        if abs(steer_angle) < 1e-6:  # Tighter threshold for straight motion
            # Straight motion
            x_new = x + v * dt * np.cos(theta)
            y_new = y + v * dt * np.sin(theta)
            theta_new = theta
            
            # Jacobian for straight motion
            F = np.eye(3)
            F[0, 2] = -v * dt * np.sin(theta)
            F[1, 2] = v * dt * np.cos(theta)
        else:
            # Circular motion with steering angle
            turn_radius = WHEELBASE / np.tan(steer_angle)
            angular_velocity = v / turn_radius
            dtheta = angular_velocity * dt
            
            # More numerically stable computation
            if abs(dtheta) < 1e-3:
                # Small angle approximation for numerical stability
                x_new = x + v * dt * np.cos(theta + dtheta/2)
                y_new = y + v * dt * np.sin(theta + dtheta/2)
            else:
                sin_theta = np.sin(theta)
                cos_theta = np.cos(theta)
                sin_theta_new = np.sin(theta + dtheta)
                cos_theta_new = np.cos(theta + dtheta)
                
                x_new = x + (v / angular_velocity) * (sin_theta_new - sin_theta)
                y_new = y - (v / angular_velocity) * (cos_theta_new - cos_theta)
            
            theta_new = theta + dtheta
            
            # Jacobian for curved motion
            F = np.eye(3)
            if abs(dtheta) > 1e-3:
                F[0, 2] = (v / angular_velocity) * (np.cos(theta + dtheta) - np.cos(theta))
                F[1, 2] = (v / angular_velocity) * (np.sin(theta + dtheta) - np.sin(theta))
        
        # Normalize angle
        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi
        
        # Update full state mean
        self.mu_full[:3] = np.array([x_new, y_new, theta_new])
        
        # Update covariance with reduced process noise
        F_full = np.eye(self.state_size)
        F_full[:3, :3] = F
        
        Q_full = np.zeros((self.state_size, self.state_size))
        Q_full[:3, :3] = self.motion_noise
        
        self.Sigma_full = F_full @ self.Sigma_full @ F_full.T + Q_full
        
        # Regularize covariance to prevent numerical issues
        self.Sigma_full = 0.5 * (self.Sigma_full + self.Sigma_full.T)
        min_var = 1e-8
        for i in range(self.state_size):
            if self.Sigma_full[i, i] < min_var:
                self.Sigma_full[i, i] = min_var
        
        self.mu = self.mu_full[:3]
        self.Sigma = self.Sigma_full[:3, :3]
        
        self.history['poses'].append(self.mu.copy())
        self.history['covariances'].append(self.Sigma.copy())
        self.history['landmarks'].append(self.landmark_positions.copy())
        
        # Decay landmark qualities
        for i in range(len(self.landmark_quality)):
            if self.landmark_initialized[i]:
                self.landmark_quality[i] *= self.quality_decay
    
    def update(self, observations):
        for landmark_id, bearing in observations:
            if landmark_id >= len(self.landmark_initialized):
                continue
                
            if not self.landmark_initialized[landmark_id]:
                self.landmark_observations[landmark_id].append(
                    (self.mu.copy(), bearing)
                )
                
                if len(self.landmark_observations[landmark_id]) > self.max_observation_history:
                    self.landmark_observations[landmark_id].pop(0)
                
                if len(self.landmark_observations[landmark_id]) >= self.landmark_init_threshold:
                    success = self.initialize_landmark_with_validation(landmark_id)
                    if success:
                        print(f"Successfully initialized landmark {landmark_id}")
                continue
                
        for landmark_id, bearing in observations:
            if landmark_id >= len(self.landmark_initialized):
                continue
                
            if not self.landmark_initialized[landmark_id]:
                continue
                
            landmark_idx = 3 + 2 * landmark_id
            lx, ly = self.mu_full[landmark_idx], self.mu_full[landmark_idx + 1]
            
            rx, ry, rtheta = self.mu
            
            dx = lx - rx
            dy = ly - ry
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > self.max_range:
                continue
                
            expected_bearing = np.arctan2(dy, dx) - rtheta
            expected_bearing = (expected_bearing + np.pi) % (2 * np.pi) - np.pi
            
            H = np.zeros((1, self.state_size))
            
            q = dx**2 + dy**2
            if q < 1e-6:  # Avoid division by zero
                continue
                
            H[0, 0] = dy / q
            H[0, 1] = -dx / q
            H[0, 2] = -1
            
            H[0, landmark_idx] = -dy / q
            H[0, landmark_idx + 1] = dx / q
            
            z = np.array([bearing])
            z_hat = np.array([expected_bearing])
            innovation = z - z_hat
            innovation = (innovation + np.pi) % (2 * np.pi) - np.pi
            
            R = np.array([[self.bearing_noise**2]])
            S = H @ self.Sigma_full @ H.T + R
            
            # Improved outlier rejection
            mahalanobis_sq = innovation @ np.linalg.inv(S) @ innovation.T
            if mahalanobis_sq > 5.0:  # Tighter threshold
                continue

            # Joseph form update for numerical stability
            K = self.Sigma_full @ H.T @ np.linalg.inv(S)
            
            self.mu_full += K @ innovation  
            I = np.eye(self.state_size)
            KH = K @ H
            self.Sigma_full = (I - KH) @ self.Sigma_full @ (I - KH).T + K @ R @ K.T
            
            # Ensure symmetry and positive definiteness
            self.Sigma_full = 0.5 * (self.Sigma_full + self.Sigma_full.T)
                    
            self.mu = self.mu_full[:3]
            self.Sigma = self.Sigma_full[:3, :3]
            
            self.landmark_positions[landmark_id] = self.mu_full[landmark_idx:landmark_idx + 2]
            self.landmark_covariances[landmark_id] = self.Sigma_full[landmark_idx:landmark_idx + 2, 
                                                                    landmark_idx:landmark_idx + 2]
            self.landmark_observed[landmark_id] = True
            
            # Increase landmark quality
            self.landmark_quality[landmark_id] = min(10, self.landmark_quality[landmark_id] + 1)
    
    def initialize_landmark_with_validation(self, landmark_id):
        observations = self.landmark_observations[landmark_id]
        if len(observations) < self.landmark_init_threshold:
            return False
            
        # NEW: Use least-squares triangulation instead of best pair
        positions = []
        weights = []
        
        for i in range(len(observations)):
            for j in range(i+1, len(observations)):
                pose1, bearing1 = observations[i]
                pose2, bearing2 = observations[j]
                
                # Global bearings
                global_bearing1 = pose1[2] + bearing1
                global_bearing2 = pose2[2] + bearing2
                
                # Baseline between observation points
                dx = pose2[0] - pose1[0]
                dy = pose2[1] - pose1[1]
                baseline = np.sqrt(dx**2 + dy**2)
                
                # Skip if baseline too small
                if baseline < self.min_baseline:
                    continue
                    
                # Angle between bearings
                angle_diff = abs(global_bearing1 - global_bearing2)
                angle_diff = min(angle_diff, 2*np.pi - angle_diff)
                
                # Check if angle is suitable
                if angle_diff < self.min_triangulation_angle or angle_diff > self.max_triangulation_angle:
                    continue
                    
                # Triangulate landmark position
                line1 = [pose1[0], pose1[1], np.cos(global_bearing1), np.sin(global_bearing1)]
                line2 = [pose2[0], pose2[1], np.cos(global_bearing2), np.sin(global_bearing2)]
                success, lx, ly = self.line_intersection(line1, line2)
                
                if success:
                    # Score based on baseline and angle
                    score = baseline * np.sin(angle_diff)
                    positions.append((lx, ly))
                    weights.append(score)
        
        if not positions:
            return False
            
        # NEW: Use weighted average of all valid positions
        positions = np.array(positions)
        weights = np.array(weights)
        total_weight = np.sum(weights)
        
        if total_weight < 1e-6:
            return False
            
        # Calculate weighted mean
        lx = np.sum(positions[:, 0] * weights) / total_weight
        ly = np.sum(positions[:, 1] * weights) / total_weight
        
        # Calculate weighted variance for covariance estimation
        var_x = np.sum(weights * (positions[:, 0] - lx)**2) / total_weight
        var_y = np.sum(weights * (positions[:, 1] - ly)**2) / total_weight
        
        # Validate position
        rx, ry, _ = self.mu
        dist = np.hypot(lx - rx, ly - ry)
        if dist > 2 * self.max_range:
            return False
            
        # Initialize landmark with covariance based on variance
        landmark_idx = 3 + 2 * landmark_id
        self.mu_full[landmark_idx] = lx
        self.mu_full[landmark_idx + 1] = ly
        
        # Set covariance based on calculated variance with minimum bound
        initial_cov = np.diag([max(var_x, 0.01), max(var_y, 0.01)]) + np.eye(2) * 0.05
        self.Sigma_full[landmark_idx:landmark_idx + 2, landmark_idx:landmark_idx + 2] = initial_cov
        
        # Update tracking
        self.landmark_positions[landmark_id] = np.array([lx, ly])
        self.landmark_covariances[landmark_id] = initial_cov
        self.landmark_initialized[landmark_id] = True
        self.landmark_observed[landmark_id] = True
        self.landmark_quality[landmark_id] = 5  # Initial quality
        
        return True
    
    def line_intersection(self, line1, line2):
        x1, y1, dx1, dy1 = line1
        x2, y2, dx2, dy2 = line2
        
        A = np.array([[dx1, -dx2], [dy1, -dy2]])
        b = np.array([x2 - x1, y2 - y1])
        
        try:
            det = np.linalg.det(A)
            if abs(det) < 1e-6:
                return False, 0, 0
                
            t = np.linalg.solve(A, b)
            x = x1 + t[0] * dx1
            y = y1 + t[0] * dy1
            return True, x, y
        except:
            return False, 0, 0
    
    def get_state(self):
        return {
            'robot_pose': self.mu.copy(),
            'robot_covariance': self.Sigma.copy(),
            'landmarks': self.landmark_positions.copy(),
            'landmark_covariances': [cov.copy() if cov is not None else None 
                                    for cov in self.landmark_covariances],
            'landmark_initialized': self.landmark_initialized.copy(),
            'history': self.history
        }


class ImprovedExplorer:
    """Improved exploration strategy focusing on baseline diversity"""
    def __init__(self, max_range=5.0, map_size=10):
        self.max_range = max_range
        self.map_size = map_size
        self.exploration_radius = 3.0
        self.center = np.array([map_size/2, map_size/2])
        self.current_target = None
        self.target_reached_threshold = 1.0
        self.exploration_points = []
        self.current_exploration_idx = 0
        self.generate_exploration_pattern()
        
    def generate_exploration_pattern(self):
        """Generate a pattern of exploration points for good baseline diversity"""
        # Create a spiral pattern for good coverage
        angles = np.linspace(0, 4*np.pi, 16)  # Two full spirals
        radii = np.linspace(1.0, self.exploration_radius, 16)
        
        self.exploration_points = []
        for angle, radius in zip(angles, radii):
            x = self.center[0] + radius * np.cos(angle)
            y = self.center[1] + radius * np.sin(angle)
            
            # Keep within bounds
            x = np.clip(x, 0.5, self.map_size - 0.5)
            y = np.clip(y, 0.5, self.map_size - 0.5)
            
            self.exploration_points.append([x, y])
        
        # Add some random points for diversity
        for _ in range(8):
            x = np.random.uniform(1, self.map_size - 1)
            y = np.random.uniform(1, self.map_size - 1)
            self.exploration_points.append([x, y])
        
        self.exploration_points = np.array(self.exploration_points)
    
    def next_target(self, robot_pose, landmarks):
        """Select next target for maximum baseline diversity"""
        rx, ry, _ = robot_pose
        
        # Check if we've reached the current target
        if self.current_target is not None:
            dist_to_target = np.hypot(self.current_target[0] - rx, self.current_target[1] - ry)
            if dist_to_target < self.target_reached_threshold:
                self.current_target = None
        
        # If no current target, select next exploration point
        if self.current_target is None:
            best_score = -float('inf')  # Initialize to negative infinity
            best_point = None
            
            # Try to find a point that gives good baseline to landmarks
            for point in self.exploration_points:
                dist_to_robot = np.hypot(point[0] - rx, point[1] - ry)
                
                # Skip if too close to current position
                if dist_to_robot < 1.5:
                    continue
                
                # Check if this point would give good baseline diversity
                baseline_score = self.calculate_baseline_score(point, robot_pose, landmarks)
                
                # Combine distance and baseline score
                total_score = baseline_score + 0.1 * dist_to_robot
                
                # Update best point if this one has a higher score
                if total_score > best_score:
                    best_score = total_score
                    best_point = point
            
            if best_point is not None:
                self.current_target = best_point
            else:
                # Fallback: pick a random point
                self.current_target = np.random.uniform([1, 1], [self.map_size-1, self.map_size-1])
        
        return self.current_target
    
    def calculate_baseline_score(self, point, robot_pose, landmarks):
        """Calculate how much baseline diversity this point would provide"""
        if len(landmarks) == 0:
            return 1.0  # Any point is good if no landmarks yet
        
        rx, ry, _ = robot_pose
        px, py = point
        
        # Calculate baseline improvement for each landmark
        total_score = 0
        for landmark in landmarks:
            lx, ly = landmark
            
            # Current baseline from robot to landmark
            current_baseline = np.hypot(lx - rx, ly - ry)
            
            # Potential baseline from point to landmark
            new_baseline = np.hypot(lx - px, ly - py)
            
            # Angle between current and new baselines
            v1 = np.array([lx - rx, ly - ry])
            v2 = np.array([lx - px, ly - py])
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_angle = np.clip(cos_angle, -1, 1)
                angle = np.arccos(cos_angle)
                
                # Score based on angle and baseline length
                baseline_score = min(current_baseline, new_baseline) * np.sin(angle)
                total_score += baseline_score
        
        return total_score


class UnknownCorrespondenceEKF_SLAM(ImprovedBearingOnlyEKF_SLAM):
    def __init__(self, initial_pose, max_landmarks, motion_noise, bearing_noise, max_range=10.0):
        super().__init__(initial_pose, max_landmarks, motion_noise, bearing_noise, max_range)
        self.data_association = DataAssociation(max_range, gate_threshold=9.21)
        self.next_landmark_id = 0
        self.max_landmarks = max_landmarks
        self.landmark_candidates = {}
        
        self.landmark_init_threshold = 3
        self.min_baseline = 1.5
        self.min_triangulation_angle = np.radians(30)
        self.max_triangulation_angle = np.radians(150)
        
        self.landmark_last_seen = [-1000] * max_landmarks
        self.candidate_min_observations = 3
        self.candidate_max_observations = 20
    def prune_uncertain_landmarks(self, current_step, max_uncertainty=3.0, min_observations=5, unseen_threshold=50):
        removed_count = 0
        
        for i in range(self.next_landmark_id):
            if not self.landmark_initialized[i]:
                continue
                
            if current_step - self.landmark_last_seen[i] < unseen_threshold:
                continue
                
            cov = self.landmark_covariances[i]
            if cov is None:
                continue
                
            eigvals = np.linalg.eigvals(cov)
            max_std = np.sqrt(np.max(eigvals))
            
            if max_std > max_uncertainty:
                if len(self.landmark_observations[i]) >= min_observations:
                    self.landmark_initialized[i] = False
                    self.landmark_positions[i] = np.zeros(2)
                    self.landmark_covariances[i] = None
                    self.landmark_observations[i] = []
                    removed_count += 1
                    
        if removed_count > 0:
            print(f"Pruned {removed_count} uncertain landmarks")
    
    def update_with_unknown_correspondences(self, raw_observations, step=0):
        if not raw_observations:
            return
            
        self.prune_uncertain_landmarks(step)
        
        observations = [(i, bearing) for i, bearing in enumerate(raw_observations)]
        
        active_landmarks = [i for i in range(self.next_landmark_id) 
                           if self.landmark_initialized[i]]
        
        associations, new_obs_indices = self.data_association.associate(
            observations, self.mu, 
            [self.landmark_positions[i] for i in active_landmarks],
            [self.landmark_covariances[i] for i in active_landmarks],
            [True] * len(active_landmarks),
            self.bearing_noise
        )
        
        for obs_idx, lm_idx in associations:
            landmark_id = active_landmarks[lm_idx]
            self.update_landmark(landmark_id, raw_observations[obs_idx])
            self.landmark_last_seen[landmark_id] = step
        
        for obs_idx in new_obs_indices:
            if self.next_landmark_id >= self.max_landmarks:
                continue
                
            bearing = raw_observations[obs_idx]
            candidate_id = self.next_landmark_id
            
            if candidate_id not in self.landmark_candidates:
                self.landmark_candidates[candidate_id] = []
            
            self.landmark_candidates[candidate_id].append(
                (self.mu.copy(), bearing))
            
            if len(self.landmark_candidates[candidate_id]) >= self.landmark_init_threshold:
                if self.initialize_candidate(candidate_id):
                    self.landmark_last_seen[candidate_id] = step
                    self.next_landmark_id += 1
                    del self.landmark_candidates[candidate_id]
    
    def initialize_candidate(self, candidate_id):
        observations = self.landmark_candidates[candidate_id]
        best_position = None
        best_score = -1
        
        for i in range(len(observations)):
            for j in range(i+1, len(observations)):
                pose1, bearing1 = observations[i]
                pose2, bearing2 = observations[j]
                
                global_bearing1 = pose1[2] + bearing1
                global_bearing2 = pose2[2] + bearing2
                
                dx = pose2[0] - pose1[0]
                dy = pose2[1] - pose1[1]
                baseline = np.sqrt(dx**2 + dy**2)
                
                if baseline < self.min_baseline:
                    continue
                
                angle_diff = abs(global_bearing1 - global_bearing2)
                angle_diff = min(angle_diff, 2*np.pi - angle_diff)
                
                if angle_diff < self.min_triangulation_angle or angle_diff > self.max_triangulation_angle:
                    continue
                
                line1 = [pose1[0], pose1[1], np.cos(global_bearing1), np.sin(global_bearing1)]
                line2 = [pose2[0], pose2[1], np.cos(global_bearing2), np.sin(global_bearing2)]
                success, lx, ly = self.line_intersection(line1, line2)
                
                if success:
                    score = baseline * np.sin(angle_diff)
                    if score > best_score:
                        best_score = score
                        best_position = (lx, ly)
        
        if best_position and best_score > 0:
            lx, ly = best_position
            rx, ry, _ = self.mu
            
            dist = np.hypot(lx - rx, ly - ry)
            if dist > self.max_range * 1.5:
                return False
                
            landmark_idx = 3 + 2 * candidate_id
            self.mu_full[landmark_idx] = lx
            self.mu_full[landmark_idx+1] = ly
            
            uncertainty_scale = 1.0 / (best_score + 1e-3)
            initial_cov = np.diag([uncertainty_scale, uncertainty_scale]) * 0.5
            
            self.Sigma_full[landmark_idx:landmark_idx+2, landmark_idx:landmark_idx+2] = initial_cov
            self.landmark_positions[candidate_id] = np.array([lx, ly])
            self.landmark_covariances[candidate_id] = initial_cov
            self.landmark_initialized[candidate_id] = True
            return True
        
        return False

    def update_landmark(self, landmark_id, bearing):
        landmark_idx = 3 + 2 * landmark_id
        lx, ly = self.mu_full[landmark_idx], self.mu_full[landmark_idx + 1]
        
        rx, ry, rtheta = self.mu
        
        dx = lx - rx
        dy = ly - ry
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist > self.max_range:
            return
            
        expected_bearing = np.arctan2(dy, dx) - rtheta
        expected_bearing = (expected_bearing + np.pi) % (2 * np.pi) - np.pi
        
        H = np.zeros((1, self.state_size))
        
        q = dx**2 + dy**2
        H[0, 0] = dy / q
        H[0, 1] = -dx / q
        H[0, 2] = -1
        
        H[0, landmark_idx] = -dy / q
        H[0, landmark_idx + 1] = dx / q
        
        z = np.array([bearing])
        z_hat = np.array([expected_bearing])
        innovation = z - z_hat
        innovation = (innovation + np.pi) % (2 * np.pi) - np.pi
        
        S = H @ self.Sigma_full @ H.T + np.array([[self.bearing_noise**2]])
        
        mahalanobis_sq = innovation @ np.linalg.inv(S) @ innovation.T
        if mahalanobis_sq > 6.0:
            return

        K = self.Sigma_full @ H.T @ np.linalg.inv(S)
        
        self.mu_full += K @ innovation  
        I = np.eye(self.state_size)
        KH = K @ H
        self.Sigma_full = (I - KH) @ self.Sigma_full @ (I - KH).T + K @ np.array([[self.bearing_noise**2]]) @ K.T
                
        self.landmark_positions[landmark_id] = self.mu_full[landmark_idx:landmark_idx + 2]
        self.landmark_covariances[landmark_id] = self.Sigma_full[landmark_idx:landmark_idx + 2, 
                                                                landmark_idx:landmark_idx + 2]
    
    def get_active_landmarks(self):
        active = []
        for i in range(self.next_landmark_id):
            if self.landmark_initialized[i]:
                active.append({
                    'id': i,
                    'position': self.landmark_positions[i].copy(),
                    'covariance': self.landmark_covariances[i].copy() if self.landmark_covariances[i] is not None else None
                })
        return active

class UnknownCorrespondenceSimulator:
    def __init__(self, landmarks, initial_pose, motion_noise, bearing_noise, max_range=10.0, fov=np.pi):
        self.landmarks = np.array(landmarks)
        self.true_pose = np.array(initial_pose)
        self.motion_noise = motion_noise
        self.bearing_noise = bearing_noise
        self.max_range = max_range
        self.fov = fov
        
        self.history = {
            'true_poses': [self.true_pose.copy()],
            'observations': []
        }
    
    def move(self, u, dt):
        v, steer_angle = u  # Using steering angle instead of angular velocity
        
        # Constrain steering to physical limits
        steer_angle = np.clip(steer_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
        
        x, y, theta = self.true_pose
        
        # Ackermann steering kinematics
        if abs(steer_angle) < 1e-5:
            x_new = x + v * dt * np.cos(theta)
            y_new = y + v * dt * np.sin(theta)
            theta_new = theta
        else:
            # Calculate turn radius from steering angle
            turn_radius = WHEELBASE / np.tan(steer_angle)
            angular_velocity = v / turn_radius
            
            x_new = x + turn_radius * (np.sin(theta + angular_velocity * dt) - np.sin(theta))
            y_new = y - turn_radius * (np.cos(theta + angular_velocity * dt) - np.cos(theta))
            theta_new = theta + angular_velocity * dt
        
        # Add noise
        x_new += np.random.normal(0, self.motion_noise[0] * dt)
        y_new += np.random.normal(0, self.motion_noise[1] * dt)
        theta_new += np.random.normal(0, self.motion_noise[2] * dt)
        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi
        
        self.true_pose = np.array([x_new, y_new, theta_new])
        self.history['true_poses'].append(self.true_pose.copy())
        return self.true_pose
    
    def sense_unknown_correspondences(self):
        observations = []
        rx, ry, rtheta = self.true_pose
        
        for i, (lx, ly) in enumerate(self.landmarks):
            dx = lx - rx
            dy = ly - ry
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > self.max_range or dist < 0.1:
                continue
                
            true_bearing = np.arctan2(dy, dx) - rtheta
            true_bearing = (true_bearing + np.pi) % (2 * np.pi) - np.pi
            
            if self.fov < 2*np.pi and abs(true_bearing) > self.fov/2:
                continue
                
            noisy_bearing = true_bearing + np.random.normal(0, self.bearing_noise)
            noisy_bearing = (noisy_bearing + np.pi) % (2 * np.pi) - np.pi
            
            observations.append(noisy_bearing)
        
        np.random.shuffle(observations)
        
        self.history['observations'].append(observations)
        return observations
    
    def get_ground_truth(self):
        return {
            'true_pose': self.true_pose.copy(),
            'landmarks': self.landmarks.copy(),
            'history': self.history
        }

def run_ackerman_simulation():
    """Run simulation with Ackermann steering model"""
    # Simulation parameters
    num_landmarks = 6
    max_landmarks = 20
    map_size = 8
    max_steps = 1500
    dt = 0.1
    
    # Create landmarks
    np.random.seed(42)
    landmarks = np.random.uniform(1.5, map_size-1.5, size=(num_landmarks, 2))
    
    # Initial robot pose
    initial_pose = np.array([map_size/2, map_size/4, 0])
    
    # Noise parameters (tuned for LEGO car)
    motion_noise = [0.01, 0.01, 0.005]  # Reduced noise for better performance
    bearing_noise = 0.01  # ~0.57 degrees
    
    # Create simulator and EKF SLAM
    sim = UnknownCorrespondenceSimulator(landmarks, initial_pose, motion_noise, bearing_noise, 
                                        max_range=4.0, fov=np.pi/2)
    ekf = UnknownCorrespondenceEKF_SLAM(initial_pose, max_landmarks, motion_noise, bearing_noise, 
                                       max_range=4.0)
    explorer = ImprovedExplorer(max_range=4.0, map_size=map_size)
    
    # Set up interactive plotting
    plt.ion()
    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111)
    
    for step in range(max_steps):
        # Get current state
        state = ekf.get_state()
        robot_pose = state['robot_pose']
        
        # Get discovered landmarks
        active_landmarks = ekf.get_active_landmarks()
        discovered_positions = np.array([lm['position'] for lm in active_landmarks]) if active_landmarks else np.empty((0, 2))
        
        # Get next target
        target = explorer.next_target(robot_pose, discovered_positions)
        
        # Calculate control command with Ackermann constraints
        rx, ry, rtheta = sim.true_pose
        dx = target[0] - rx
        dy = target[1] - ry
        
        target_theta = np.arctan2(dy, dx)
        theta_error = (target_theta - rtheta + np.pi) % (2*np.pi) - np.pi
        
        # Smooth control with Ackermann constraints
        max_linear_vel = MAX_SPEED
        turn_factor = max(0.15, 1.0 - abs(theta_error) / np.pi)
        v = max_linear_vel * turn_factor
        
        # Ensure minimum speed for turning
        if abs(theta_error) > 0.1:
            v = max(v, 0.1)
        
        # Calculate steering angle based on error
        steer_angle = 0.8 * theta_error
        
        # Constrain steering angle
        steer_angle = np.clip(steer_angle, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
        
        # Simulate robot motion
        sim.move([v, steer_angle], dt)
        
        # Get sensor observations
        raw_observations = sim.sense_unknown_correspondences()
        
        # EKF steps
        ekf.predict([v, steer_angle], dt)
        ekf.update_with_unknown_correspondences(raw_observations, step=step)
        
        # Update visualization
        if step % 8 == 0:
            ax.clear()
            
            # Ground truth landmarks
            gt_landmarks = sim.get_ground_truth()['landmarks']
            ax.scatter(gt_landmarks[:, 0], gt_landmarks[:, 1], c='green', marker='s', 
                      s=200, alpha=0.9, label='True Landmarks', edgecolors='black', linewidth=2)
            
            # Label ground truth landmarks
            for i, (x, y) in enumerate(gt_landmarks):
                ax.annotate(f'GT{i}', (x, y), xytext=(5, 5), textcoords='offset points', 
                           fontsize=8, color='darkgreen', weight='bold')
            
            # Estimated landmarks
            active_landmarks = ekf.get_active_landmarks()
            for lm in active_landmarks:
                x, y = lm['position']
                ax.scatter(x, y, c='red', marker='x', s=200, linewidth=4,
                          label='Estimated Landmarks' if lm['id'] == 0 else "")
                
                # Label estimated landmarks
                ax.annotate(f"L{lm['id']}", (x, y), xytext=(5, -15), textcoords='offset points', 
                           fontsize=8, color='darkred', weight='bold')
                
                # Plot covariance ellipse
                cov = lm['covariance']
                if cov is not None:
                    try:
                        eigvals, eigvecs = np.linalg.eig(cov)
                        if np.all(eigvals > 0):
                            angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
                            width, height = 4 * np.sqrt(eigvals)
                            ellipse = Ellipse((x, y), width, height, angle=angle, 
                                             fill=False, color='red', alpha=0.7, linewidth=2)
                            ax.add_patch(ellipse)
                    except:
                        pass
            
            # Robot paths
            gt_poses = np.array(sim.get_ground_truth()['history']['true_poses'])
            ax.plot(gt_poses[:, 0], gt_poses[:, 1], 'g-', linewidth=3, label='True Path', alpha=0.8)
            
            est_poses = np.array(state['history']['poses'])
            ax.plot(est_poses[:, 0], est_poses[:, 1], 'b--', linewidth=2, label='Estimated Path', alpha=0.8)
            
            # Current robot positions
            current_gt_pose = sim.get_ground_truth()['true_pose']
            current_est_pose = state['robot_pose']
            
            # Draw robot with steering visualization
            draw_robot(ax, current_gt_pose, 'green')
            draw_robot(ax, current_est_pose, 'blue')
            
            # Current target
            if explorer.current_target is not None:
                ax.scatter(explorer.current_target[0], explorer.current_target[1], 
                          c='magenta', marker='*', s=250, label='Current Target', edgecolors='black')
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'Bearing-Only SLAM with Ackermann Steering (Step {step})\n'
                        f'True Landmarks: {num_landmarks}, Estimated: {len(active_landmarks)}', fontsize=14)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(-0.5, map_size + 0.5)
            ax.set_ylim(-0.5, map_size + 0.5)
            ax.set_aspect('equal')
            
            plt.pause(0.01)
        
        # Print status
        num_active = len(active_landmarks)
        print(f"Step {step}: True landmarks: {num_landmarks}, Estimated: {num_active}, "
              f"Observations: {len(raw_observations)}", end='\r')
    
    plt.ioff()
    plt.show()
    
    return ekf, sim

def draw_robot(ax, pose, color):
    """Draw robot with steering visualization"""
    x, y, theta = pose
    length = 0.6
    width = 0.4
    
    # Draw robot body
    rect = Rectangle((x - length/2, y - width/2), length, width, 
                    angle=np.degrees(theta), rotation_point='center',
                    fill=False, color=color, linewidth=2)
    ax.add_patch(rect)
    
    # Draw wheels
    wheel_length = 0.15
    wheel_width = 0.08
    
    # Front wheels (steered)
    wheel_angle = np.radians(30)  # Fixed angle for visualization
    wheel_positions = [
        (x + length/3, y + width/2, theta + wheel_angle),
        (x + length/3, y - width/2, theta - wheel_angle)
    ]
    
    for wx, wy, wangle in wheel_positions:
        wheel = Rectangle((wx - wheel_length/2, wy - wheel_width/2), 
                         wheel_length, wheel_width,
                         angle=np.degrees(wangle), rotation_point='center',
                         fill=True, color='black', alpha=0.7)
        ax.add_patch(wheel)
    
    # Rear wheels
    wheel_positions = [
        (x - length/3, y + width/2, theta),
        (x - length/3, y - width/2, theta)
    ]
    
    for wx, wy, wangle in wheel_positions:
        wheel = Rectangle((wx - wheel_length/2, wy - wheel_width/2), 
                         wheel_length, wheel_width,
                         angle=np.degrees(wangle), rotation_point='center',
                         fill=True, color='black', alpha=0.7)
        ax.add_patch(wheel)
    
    # Draw wheelbase
    ax.plot([x - length/3, x + length/3], [y, y], 'k-', linewidth=1)
    
    # Draw steering direction
    turn_radius = WHEELBASE / np.tan(wheel_angle)
    if abs(turn_radius) < 10:  # Only draw if turning radius is reasonable
        center_x = x - turn_radius * np.sin(theta)
        center_y = y + turn_radius * np.cos(theta)
        
        # Draw turning circle
        turn_circle = plt.Circle((center_x, center_y), abs(turn_radius), 
                               fill=False, color='gray', linestyle='--', alpha=0.5)
        ax.add_patch(turn_circle)

# Run the simulation
print("Starting bearing-only SLAM simulation with Ackermann steering...")
ekf, sim = run_ackerman_simulation()