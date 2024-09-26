        # Camera intrinsic parameters
        self.camera_matrix = np.array([[761.7297, 0, 640.3348],
                                       [0, 762.2555, 362.3959],
                                       [0, 0, 1.0000]])

        # Distortion coefficients
        self.dist_coeffs = np.array([-0.0079, -0.00062538, 0, 0, 0])

        # Extrinsic parameters (rotation and translation)
        self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
                                         [-0.2391, -0.0467, -0.9699],
                                         [0.9710, -0.0137, -0.2387]])

        self.translation_vector = np.array([0.1116, -0.3260, -0.1386])