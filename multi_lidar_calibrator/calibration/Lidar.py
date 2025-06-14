import numpy as np
import open3d as o3d
from geometry_msgs.msg import Transform

from .Geometry import Rotation, TransformationMatrix, Translation


class Lidar:
    """
    Represents a LiDAR sensor with associated point cloud data and transformation matrices.
    """

    def __init__(self, name: str, translation: Translation, rotation: Rotation):
        """
        Initialize a Lidar object.

        Args:
            name: The name of the Lidar sensor.
            translation: A Translation object representing the translation component of the sensor's pose.
            rotation: A Rotation object representing the rotation component of the sensor's pose.

        Returns:
            None
        """
        self.name = name
        self.translation = translation
        self.rotation = rotation
        self.tf_matrix = TransformationMatrix(translation, rotation)
        self.pcd = None
        self.pcd_transformed = None

    @classmethod
    def from_transform(cls, name: str, transform: Transform):
        """
        Create a Lidar object from a ROS Transform message.

        Args:
            name: The name of the Lidar sensor.
            transform: A ROS Transform message representing the sensor's pose.

        Returns:
            A Lidar object.
        """
        translation = Translation(
            transform.translation.x, transform.translation.y, transform.translation.z
        )
        rotation = Rotation.from_quaternion(
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        )
        return cls(name, translation, rotation)

    def read_pcd(self, path: str):
        """
        Read a point cloud from a file and store it in the Lidar object.

        Args:
            path: The path to the point cloud file.

        Returns:
            None
        """
        self.pcd = o3d.io.read_point_cloud(path)
        self.pcd_transformed = o3d.geometry.PointCloud(self.pcd)  # Deep copy

    def load_pcd(self, pcd):
        """
        Load a point cloud into the Lidar object.

        Args:
            pcd: An Open3D PointCloud object.

        Returns:
            None
        """
        self.pcd = o3d.geometry.PointCloud(pcd)
        self.pcd_transformed = o3d.geometry.PointCloud(self.pcd)

    def remove_ground_plane(
        self, pcd=None, distance_threshold=0.1, ransac_n=3, num_iterations=1000
    ):
        """
        Remove the ground plane from a point cloud using RANSAC.

        Args:
            pcd: An Open3D PointCloud object. If None, the point cloud of the Lidar object is used.
            distance_threshold: The distance threshold for the RANSAC algorithm.
            ransac_n: The number of points to sample for the RANSAC algorithm.
            num_iterations: The number of iterations for the RANSAC algorithm.

        Returns:
            An Open3D PointCloud object with the ground plane removed.
        """
        if pcd is None:
            pcd = self.pcd
        plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        return outlier_cloud

    def calibrate_pitch(self, d_th=0.1, ransac_n=3, num_ransac_iter=1000, v_s=0.5, runs=5):
        """
        Calibrate the pitch angle between the LiDAR and the ground plane. This method assumes that the yaw angle is accurate and ignores it.

        Args:
            d_th: The distance threshold for the RANSAC algorithm.
            ransac_n: The number of points to sample for the RANSAC algorithm.
            num_ransac_iter: The number of iterations for the RANSAC algorithm.
            v_s: The voxel size for downsampling the point cloud. If 0, no downsampling is performed.
            runs: The number of times to run the calibration.

        Returns:
            The calibrated roll and pitch angles.
        """
        roll, pitch = np.zeros(runs), np.zeros(runs)
        for i in range(runs):
            pcd = o3d.geometry.PointCloud(self.pcd)
            if v_s > 0.0:
                pcd = pcd.voxel_down_sample(v_s)
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=d_th, ransac_n=ransac_n, num_iterations=num_ransac_iter
            )
            [a, b, c, d] = plane_model
            # Compute Euler angles (yaw is not applicable)
            roll[i] = np.arctan2(b, c)
            pitch[i] = -np.arctan2(a, np.sqrt(b**2 + c**2))

        return [np.median(roll), np.median(pitch)]
    
    # def calibrate_pitch(self, d_th=0.1, ransac_n=3, num_ransac_iter=1000, v_s=0.5, runs=5, visualize_first_run=True):
    #     """
    #     Calibrate the pitch angle between the LiDAR and the ground plane. This method assumes that the yaw angle is accurate and ignores it.

    #     Args:
    #         d_th: The distance threshold for the RANSAC algorithm.
    #         ransac_n: The number of points to sample for the RANSAC algorithm.
    #         num_ransac_iter: The number of iterations for the RANSAC algorithm.
    #         v_s: The voxel size for downsampling the point cloud. If 0, no downsampling is performed.
    #         runs: The number of times to run the calibration.
    #         visualize_first_run: If True, visualizes the detected plane and points from the first run.
    #     Returns:
    #         A list containing the median calibrated roll and pitch angles in radians.
    #     """
    #     roll_results, pitch_results = np.full(runs, np.nan), np.full(runs, np.nan) # Initialize with NaN
        
    #     if self.pcd is None or not self.pcd.has_points():
    #         print("ERROR: Lidar.calibrate_pitch - Point cloud is not loaded or is empty.")
    #         return [np.nan, np.nan]

    #     for i in range(runs):
    #         pcd_for_run = o3d.geometry.PointCloud(self.pcd) # Make a deep copy for each run
            
    #         if v_s > 0.0:
    #             pcd_for_run = pcd_for_run.voxel_down_sample(v_s)
            
    #         if not pcd_for_run.has_points() or len(pcd_for_run.points) < ransac_n:
    #             message = f"Run {i+1}/{runs}: Not enough points ({len(pcd_for_run.points) if pcd_for_run.has_points() else 0}) " \
    #                       f"for RANSAC (need {ransac_n})."
    #             if v_s > 0.0: message += f" (Voxel size: {v_s})"
    #             print(f"WARNING: Lidar.calibrate_pitch - {message} Skipping this run.")
                
    #             if i == 0 and visualize_first_run and pcd_for_run.has_points():
    #                 print("  Visualizing the (insufficient) point cloud.")
    #                 o3d.visualization.draw_geometries([pcd_for_run], 
    #                                                   window_name=f"Run {i+1} - Insufficient Points ({len(pcd_for_run.points)})")
    #             continue # Results for this run remain NaN

    #         plane_model, inliers = pcd_for_run.segment_plane(
    #             distance_threshold=d_th, ransac_n=ransac_n, num_iterations=num_ransac_iter
    #         )

    #         if len(inliers) < ransac_n: 
    #             print(f"WARNING: Lidar.calibrate_pitch - Run {i+1}/{runs}: RANSAC found only {len(inliers)} inliers "
    #                   f"(threshold {ransac_n}). Skipping this run.")
    #             if i == 0 and visualize_first_run:
    #                 print("  Visualizing point cloud as no sufficient plane was found by RANSAC.")
    #                 if pcd_for_run.has_points():
    #                      o3d.visualization.draw_geometries([pcd_for_run], 
    #                                                        window_name=f"Run {i+1} - RANSAC Failed (Inliers: {len(inliers)})")
    #             continue # Results for this run remain NaN

    #         [a, b, c, d] = plane_model
    #         roll_results[i] = np.arctan2(b, c)
    #         pitch_results[i] = -np.arctan2(a, np.sqrt(b**2 + c**2))

    #         if visualize_first_run:
    #             print(f"INFO: Lidar.calibrate_pitch - Visualizing detected plane from run {i+1}/{runs}:")
    #             print(f"  Point cloud for this run has {len(pcd_for_run.points)} points.")
    #             if v_s > 0: print(f"  (Used voxel size: {v_s})")
    #             print(f"  Plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
    #             print(f"  Number of inliers: {len(inliers)}")
    #             print(f"  Calculated roll (this run): {np.rad2deg(roll_results[i]):.2f} deg")
    #             print(f"  Calculated pitch (this run): {np.rad2deg(pitch_results[i]):.2f} deg")
                
    #             inlier_cloud = pcd_for_run.select_by_index(inliers)
    #             outlier_cloud = pcd_for_run.select_by_index(inliers, invert=True)
                
    #             geometries_to_draw = []
    #             if inlier_cloud.has_points():
    #                 inlier_cloud.paint_uniform_color([1, 0, 0])  # Red for plane inliers
    #                 geometries_to_draw.append(inlier_cloud)
                
    #             if outlier_cloud.has_points():
    #                 outlier_cloud.paint_uniform_color([0.8, 0.8, 0.8]) # Gray for outliers
    #                 geometries_to_draw.append(outlier_cloud)
                
    #             if not geometries_to_draw and pcd_for_run.has_points(): # Fallback: show original if selections failed
    #                 print("  WARNING: Could not prepare inlier/outlier clouds for visualization, showing processed PCD.")
    #                 geometries_to_draw.append(pcd_for_run)

    #             if geometries_to_draw:
    #                 o3d.visualization.draw_geometries(geometries_to_draw, 
    #                                                   window_name=f"Run {i+1} - Detected Plane (Red = Inliers)")
    #             else:
    #                 print(f"  INFO: No geometries to draw for run {i+1} visualization (PCD might be empty).")
        
    #     valid_rolls = roll_results[~np.isnan(roll_results)]
    #     valid_pitches = pitch_results[~np.isnan(pitch_results)]

    #     if len(valid_rolls) == 0 or len(valid_pitches) == 0: # Check if any valid results were obtained
    #         print("WARNING: Lidar.calibrate_pitch - No valid calibration results obtained from any run. Returning [NaN, NaN].")
    #         return [np.nan, np.nan]
            
    #     final_roll = np.median(valid_rolls)
    #     final_pitch = np.median(valid_pitches)
        
    #     print(f"INFO: Lidar.calibrate_pitch - Median Roll: {np.rad2deg(final_roll):.2f} deg, "
    #           f"Median Pitch: {np.rad2deg(final_pitch):.2f} deg (from {len(valid_rolls)} valid runs out of {runs}).")
    #     return [final_roll, final_pitch]
