
import math

class wall_or_cube:
    def __init__(self, distance_threshold=50, wall_width_threshold=30, cube_width=2.66):
        """
        Initialize thresholds for classifying objects.

        :param distance_threshold: Maximum distance to consider an object (cm).
        :param wall_width_threshold: Minimum width to classify as a wall (cm).
        :param cube_width: Expected width of a cube (cm).
        """
        self.distance_threshold = distance_threshold
        self.wall_width_threshold = wall_width_threshold
        self.cube_width = cube_width

    def classify_object(self, cluster, distances):
        """
        Classify an object as a wall or a cube based on its angular span and distance.

        :param cluster: List of angles in the cluster.
        :param distances: List of distances corresponding to the cluster.
        :return: "wall", "cube", or "unknown".
        """
        angular_span = abs(cluster[-1] - cluster[0])  # Difference between first and last angle
        avg_distance = sum(distances) / len(distances)

        if avg_distance > self.distance_threshold:
            return "unknown"  # Too far to classify

        # Calculate approximate width using the angular span and average distance
        width = 2 * avg_distance * math.tan(math.radians(angular_span / 2))

        # Classification based on width
        if width >= self.wall_width_threshold:
            return "wall"
        elif abs(width - self.cube_width) <= 1:  # Allowing a small tolerance
            return "cube"
        else:
            return "unknown"

    def find_clusters(self, distances):
        """
        Identify clusters of close distances from a list.

        :param distances: List of distances corresponding to angles (0–180).
        :return: List of clusters, where each cluster is a list of indices (angles).
        """
        clusters = []
        cluster = []
        threshold = 10  # Threshold for considering distances part of the same cluster

        for angle, distance in enumerate(distances):
            if distance < self.distance_threshold:  # Valid distance reading
                if cluster and abs(distance - distances[cluster[-1]]) > threshold:
                    clusters.append(cluster)
                    cluster = []
                cluster.append(angle)
            elif cluster:  # End of a cluster
                clusters.append(cluster)
                cluster = []

        if cluster:  # Append any remaining cluster
            clusters.append(cluster)

        return clusters

# Example Usage
if __name__ == "__main__":
    analyzer = wall_or_cube()

    # Simulated distance readings (angles 0–180)
    distances = [float('inf')] * 181
 

    # Find clusters of objects
    clusters = analyzer.find_clusters(distances)

    # Classify each cluster
    for cluster in clusters:
        cluster_distances = [distances[angle] for angle in cluster]
        classification = analyzer.classify_object(cluster, cluster_distances)
        print(f"Cluster at angles {cluster} classified as: {classification}")
