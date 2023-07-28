import json
import numpy as np
import cv2
from .camera import Camera


class Calibration(object):
    def __init__(self, cameras, selected_camera_ids=None):
        self.cameras = {k: Camera.from_dict(v) for k, v in cameras.items()}
        if selected_camera_ids is not None:
            self.cameras = {
                camera_id: self.cameras[camera_id] for camera_id in selected_camera_ids
            }

        self.camera_ids = tuple(self.cameras.keys())
        self.world_ltrb = self.compute_world_ltrb()

    def get_projection_matrix(self, camera_id):
        camera = self.cameras[camera_id]
        P = camera.K @ np.eye(3, 4) @ np.linalg.inv(camera.Tw)
        return P

    def compute_world_ltrb(self, increment=0):
        cameras = self.cameras
        camera_ids = list(dict(cameras).keys())

        xs = []
        ys = []
        for camera_id in camera_ids:
            Tw = cameras[camera_id].Tw
            xs.append(Tw[0, 3])
            ys.append(Tw[1, 3])

        x1, y1 = min(xs) - increment, min(ys) - increment
        x2, y2 = max(xs) + increment, max(ys) + increment

        return [x1, y1, x2, y2]

    def undistort(self, point_2d, camera_id):
        camera = self.cameras[camera_id]
        point_2d = cv2.undistortPoints(
            point_2d, camera.K, camera.dist_coeffs, P=camera.K
        ).squeeze(axis=1) #.reshape(-1)
        #point_2d = np.hstack((point_2d, np.ones((point_2d.shape[0], 1))))
       
        return point_2d

    def project(self, points_3d, camera_id):
        camera = self.cameras[camera_id]  # type: Camera
        return camera.project(points_3d)

    def triangulate(self, points_2d, camera_ids,image_wh):
        """
        Triangulation on multiple points from different cameras.
        args:
            points_2d: N x 2 np.ndarray of 2D points,
                       the points should be normalized by the image width and height,
                       i.e. the inputed x, y should be in the range of [0, 1]
            camera_ids: camera id for each point comes from
        """
        assert len(points_2d) >= 2, "triangulation requires at least two cameras"
        
        points_2d = np.asarray(points_2d)
        for i in range(len(points_2d)):
            points_2d[i,0] = points_2d[i,0]/image_wh[i][0]
            points_2d[i,1] = points_2d[i,1]/image_wh[i][1]
        A = np.zeros([len(points_2d) * 2, 4], dtype=float)
        for i, point in enumerate(points_2d):
            camera_id = camera_ids[i]
            upoint = self.undistort(point, camera_id).reshape(-1)
            P = self.get_projection_matrix(camera_id)
            P3T = P[2]
            A[2 * i, :] = upoint[0] * P3T - P[0]
            A[2 * i + 1, :] = upoint[1] * P3T - P[1]
        u, s, vh = np.linalg.svd(A)
        error = s[-1]
        X = vh[len(s) - 1]
        point_3d = X[:3] / X[3]

        return error, point_3d

    def triangulate_complete_pose(self, n_cameras_points_2d, camera_ids, image_wh):
        """ 
        Triangulation on multiple points from different cameras.
        args:
            n_cameras_points_2d : N cameras x K body joints x 2 np.ndarray of 2D points
            camera_ids: camera id for each point comes from
            
        """
        
        n_cameras_points_2d = np.array(n_cameras_points_2d)
        n_cameras = n_cameras_points_2d.shape[0]
        points_3d = np.zeros((n_cameras_points_2d.shape[1], 3))
        for index in range(n_cameras_points_2d.shape[1]):
            error, points_3d[index, :] = self.triangulate(n_cameras_points_2d[:,index,:].reshape(n_cameras,2), camera_ids, image_wh)
        
        return points_3d
        
        
        
    def linear_ls_triangulate_weighted(self, points_2d, camera_ids, image_wh, lambda_t, timestamps):
        """
        Triangulation on multiple points from different cameras.
        args:
            points_2d: N x 2 np.ndarray of 2D points,
                       
            
            camera_ids: camera id for each point comes from
            image_wh: N x 2 to normalize points by the image width and height,
            lambda_t: scalar to calc weights
            timestamps: N vector to calc weights given according to camera ID list
                       
        """
        assert len(points_2d) >= 2, "triangulation requires at least two cameras"
        
        
        points_2d = np.asarray(points_2d)
        for i in range(len(points_2d)):
            points_2d[i,0] = points_2d[i,0]/image_wh[i][0]
            points_2d[i,1] = points_2d[i,1]/image_wh[i][1]
        A = np.zeros([len(points_2d) * 2, 4], dtype=float)
        weighted_A = np.zeros([len(points_2d) * 2, 4], dtype=float)
        # by defination t will be the latest time
        t = max(timestamps)
        for i, point in enumerate(points_2d):
            camera_id = camera_ids[i]
            upoint = self.undistort(point, camera_id).reshape(-1)
            P = self.get_projection_matrix(camera_id)
            P3T = P[2]
            A[2 * i, :] = upoint[0] * P3T - P[0]
            A[2 * i + 1, :] = upoint[1] * P3T - P[1]
            
            weight_time = np.exp(-lambda_t * (t - timestamps[i]))
            weighted_A[2 * i, :] = A[2 * i, :] *  (weight_time/np.linalg.norm(A[2 * i, :]))
            weighted_A[2 * i + 1, :] = A[2 * i + 1, :] *  (weight_time/np.linalg.norm(A[2 * i + 1, :]))

        u, s, vh = np.linalg.svd(weighted_A)
        error = s[-1]
        X = vh[len(s) - 1]
        # The final non-homogeneous coordinate Xt can be obtained by dividing the homogeneous coordinate X˜t 
        # by its fourth value: Xt = X˜t/(X˜t)4.
        point_3d = X[:3] / X[3]

        return error, point_3d
    
    def moveExtrinsicOriginToFirstCamera(self,R1,R2,t1,t2):
        """
        Center extrinsic parameters world coordinate system into camera 1.
        
        Compute R (rotation from camera 1 to camera 2) and T (translation from camera 1 to camera 2) as used in OpenCV
        from extrinsic of two cameras centered anywhere else.
        This is particularly useful when the world coordinate system is not centered into the first camera.
        
        Parameters
        ----------
        R1, R2 : np.array
            3x3 rotation matrices that go from world origin to each camera center.
        t1, t2 : np.array
            3x1 translation vectors that go from world origin to each camera center.
            
        Returns
        -------
        numpy.ndarray
            Rotation matrix between the 1st and the 2nd camera coordinate systems as numpy.ndarray.
        numpy.ndarray
            Translation vector between the coordinate systems of the cameras as numpy.ndarray.
        """
        t1 = t1.reshape((-1,1)) # Force vertical shape
        t2 = t2.reshape((-1,1))
        R = R2.dot(np.transpose(R1))
        t = t2 - R2.dot(np.transpose(R1)).dot(t1)
        return R, t
    
    def getCrossProductMatrix(self, v):
        """
        Build the 3x3 antisymmetric matrix representing the cross product with v.
        
        In literature this is often indicated as [v]\ :subscript:`x`.
        
        Parameters
        ----------
        v : numpy.ndarray or list
            A 3-dimensional vector.
        
        Returns
        -------
        numpy.ndarray
            A 3x3 matrix representing the cross product with the input vector.
        """
        v = v.ravel()
        return np.array( [ [0, -v[2], v[1]], \
                        [v[2], 0, -v[0]], \
                        [-v[1], v[0], 0] ] , dtype=float)
        
    def get_fundamental_matrix(self, camera_ids):
        """ 
        camera_ids: camera id for each point comes from
        """
        
        extrinsic_mat_1 = np.linalg.inv(self.cameras[camera_ids[0]].Tw)
        extrinsic_mat_2 = np.linalg.inv(self.cameras[camera_ids[1]].Tw)
        R1 = extrinsic_mat_1[:3, :3]
        t1 = extrinsic_mat_1[:3, 3].reshape(-1,1) / 100
        R2 = extrinsic_mat_2[:3, :3]
        t2 = extrinsic_mat_2[:3, 3].reshape(-1,1) / 100 # comparing with T vector from original dataset the values here are scaled by factor of 100 
        
        R,t = self.moveExtrinsicOriginToFirstCamera(R1,R2,t1,t2)
        
        # refer https://github.com/zju3dv/mvpose/issues/60 for scale information
        K1 = self.cameras[camera_ids[0]].unnormalized(288) # returns unnnormalized K with height h and width w = self.aspect * h
        K2 = self.cameras[camera_ids[1]].unnormalized(288)
        
        vv = self.getCrossProductMatrix(K1.dot(np.transpose(R)).dot(t))
        F = np.transpose(np.linalg.inv(K2)).dot(R).dot(np.transpose(K1)).dot(vv)
        return F    
    
    def fundamental_from_projections(self, camera_ids):
        """Get the Fundamental matrix from Projection matrices.

        Args:
            P1: The projection matrix from first camera with shape :math:`( 3, 4)`.
            P2: The projection matrix from second camera with shape :math:`( 3, 4)`.

        Returns:
            The fundamental matrix with shape :math:`(3, 3)`.
        """
        P1 = self.get_projection_matrix(camera_ids[0])
        P2 = self.get_projection_matrix(camera_ids[1])
        
        if not (len(P1.shape) >= 2 and P1.shape[-2:] == (3, 4)):
            raise AssertionError(P1.shape)
        if not (len(P2.shape) >= 2 and P2.shape[-2:] == (3, 4)):
            raise AssertionError(P2.shape)
        if P1.shape[:-2] != P2.shape[:-2]:
            raise AssertionError

        def vstack(x, y):
            return np.concatenate([x, y], axis=0)

        X1 = P1[..., 1:, :]
        X2 = vstack(P1[..., 2:3, :], P1[..., 0:1, :])
        X3 = P1[..., :2, :]

        Y1 = P2[..., 1:, :]
        Y2 = vstack(P2[..., 2:3, :], P2[..., 0:1, :])
        Y3 = P2[..., :2, :]

        X1Y1, X2Y1, X3Y1 = vstack(X1, Y1), vstack(X2, Y1), vstack(X3, Y1)
        X1Y2, X2Y2, X3Y2 = vstack(X1, Y2), vstack(X2, Y2), vstack(X3, Y2)
        X1Y3, X2Y3, X3Y3 = vstack(X1, Y3), vstack(X2, Y3), vstack(X3, Y3)
        
        F_vec = np.array(
            [[np.linalg.det(X1Y1).reshape(-1), np.linalg.det(X2Y1).reshape(-1), np.linalg.det(X3Y1).reshape(-1)],
            [ np.linalg.det(X1Y2).reshape(-1), np.linalg.det(X2Y2).reshape(-1), np.linalg.det(X3Y2).reshape(-1)],
            [ np.linalg.det(X1Y3).reshape(-1), np.linalg.det(X2Y3).reshape(-1), np.linalg.det(X3Y3).reshape(-1)]]
            )
       
        #return F_vec.view(*P1.shape[:-2], 3, 3)
        return F_vec.reshape(3,3)


    def convert_points_to_homogeneous(self, points):
        """Function that converts points from Euclidean to homogeneous space.

        Args:
            points: the points to be transformed with shape :math:`(N, D)`.

        Returns:
            the points in homogeneous coordinates :math:`(N, D+1)`.

        """
        #assert len(points.shape) < 2, f"Input must be at least a 2D tensor. Got {points.shape}"

        return np.hstack((points, np.ones((points.shape[0], 1))))

     
    def point_line_distance(self, point, line, eps: float = 1e-9):
            
            """ 
            Args:
            point: (possibly homogeneous) points :math:`(N, 2 or 3)`.
            line: lines coefficients :math:`(a, b, c)` with shape :math:`(N, 3)`, where :math:`ax + by + c = 0`.
            eps: Small constant for safe sqrt.

            Returns:
                the computed distance with shape :math:`(N)`.
            
            """
            
            
            numerator = abs(line[:, 0] * point[:, 0] + line[:, 1] * point[:, 1] + line[:, 2])
            denominator = np.sqrt(line[:, 0]*line[:, 0] + line[:, 1]*line[:, 1])
            #print(numerator / (denominator + eps))
            return numerator / (denominator + eps)
    
    def left_to_right_epipolar_distance(self, pts1, pts2, Fm):
        """Return one-sided epipolar distance for correspondences given the fundamental matrix.

        This method measures the distance from points in the right images to the epilines
        of the corresponding points in the left images as they reflect in the right images.

        Args:
        pts1: correspondences from the left images with shape
            :math:`(1,  3)`. If they are not homogeneous, converted automatically.
        pts2: correspondences from the right images with shape
            :math:`(1,  3)`. If they are not homogeneous, converted automatically.
        Fm: Fundamental matrices with shape :math:`(3, 3)`. Called Fm to
            avoid ambiguity with torch.nn.functional.

        Returns:
            the computed Symmetrical distance with shape :math:`(1)`.
        """
        #F_t = Fm.transpose()
        #line1_in_2 =  np.matmul(pts1,F_t)
        #line1_in_2 =  np.dot(pts1,F_t)
        #line1_in_2 = np.matmul(Fm, pts1.transpose()).transpose()
            
        # line on Right image = F * pts1
        
        # here pts1 and pts2 are in shape (N,3) so we actually receive tranposed points
        # To fit to eq we apply changes accordingly here
        
        F_t = Fm.transpose()
        line1_in_2 = np.matmul(pts1,F_t)  
        
        #print(f'Point R: {pts2}')
        #print(f'Epipolar in R: {line1_in_2}')
        
        # since point_line_distance also expect inputs in shape (N,3)
        return self.point_line_distance(pts2, line1_in_2)



    def right_to_left_epipolar_distance(self, pts1, pts2, Fm):
        """Return one-sided epipolar distance for correspondences given the fundamental matrix.

        This method measures the distance from points in the left images to the epilines
        of the corresponding points in the right images as they reflect in the left images.

        Args:
        pts1: correspondences from the left images with shape
            :math:`(1, 3)`. If they are not homogeneous, converted automatically.
        pts2: correspondences from the right images with shape
            :math:`(1, 3)`. If they are not homogeneous, converted automatically.
        Fm: Fundamental matrices with shape :math:`(3, 3)`. Called Fm to
            avoid ambiguity with torch.nn.functional.

        Returns:
            the computed Symmetrical distance with shape :math:`(1)`.
        """
        
        # line on Left image  = F.transpose() * pts2
        # here pts1 and pts2 are in shape (N,3) so we actually receive tranposed points
        # To fit to eq we apply changes accordingly here
        
        line2_in_1 = np.matmul(pts2 , Fm)
        #print(f'Point L: {pts1}')
        #print(f'Epipolar in L: {line2_in_1}')
        
        # since point_line_distance also expect inputs in shape (N,3)
        return self.point_line_distance(pts1, line2_in_1)
   
    
    def distance_between_epipolar_lines(self, correspondence1, correspondence2,  fundamental_matrix):
        
       
        #undistored_point_1 = np.array(self.undistort(correspondence1, camera_ids[0]))
        #undistored_point_2 = np.array(self.undistort(correspondence2, camera_ids[1]))
        
        point1 = self.convert_points_to_homogeneous(correspondence1)
        point2 = self.convert_points_to_homogeneous(correspondence2)
        
        #this function will expect unnormalized points 1 and points 2
        dist_1 = np.mean(self.right_to_left_epipolar_distance(point1,point2,fundamental_matrix))
        dist_2 = np.mean(self.left_to_right_epipolar_distance(point1,point2,fundamental_matrix))
        
        
        #print(self.right_to_left_epipolar_distance(point1,point2,fundamental_matrix), dist_1)
        #print(self.left_to_right_epipolar_distance(point1,point2,fundamental_matrix), dist_2)
            
        distance = dist_1 + dist_2 
        return distance
    
    def sampson_epipolar_distance(self, pts1, pts2, camera_ids, Fm, squared = True, eps = 1e-8):
        """Return Sampson distance for correspondences given the fundamental matrix.

        Args:
            pts1: correspondences from the left images with shape :math:`(*, N, (2|3))`. If they are not homogeneous,
                converted automatically.
            pts2: correspondences from the right images with shape :math:`(*, N, (2|3))`. If they are not homogeneous,
                converted automatically.
            Fm: Fundamental matrices with shape :math:`(*, 3, 3)`. Called Fm to avoid ambiguity with torch.nn.functional.
            squared: if True (default), the squared distance is returned.
            eps: Small constant for safe sqrt.

        Returns:
            the computed Sampson distance with shape :math:`(*, N)`.
        """
        
        pts1 = np.array(self.undistort(pts1, camera_ids[0]))
        pts2 = np.array(self.undistort(pts2, camera_ids[1]))
        
        if pts1.shape[-1] == 2:
            pts1 = self.convert_points_to_homogeneous(pts1)

        if pts2.shape[-1] == 2:
            pts2 = self.convert_points_to_homogeneous(pts2)

        K1 = self.cameras[camera_ids[0]].K.copy()
        K2 = self.cameras[camera_ids[1]].K.copy()
        
        K1_inv = np.linalg.inv(K1)
        K2_inv = np.linalg.inv(K2)
        
        pts1 = np.dot(K1_inv, pts1.transpose() ).transpose()   # Shape (14,3)
        pts2 = np.dot(K2_inv, pts2.transpose() ).transpose()
        # From Hartley and Zisserman, Sampson error (11.9)
        # sam =  (x'^T F x) ** 2 / (  (((Fx)_1**2) + (Fx)_2**2)) +  (((F^Tx')_1**2) + (F^Tx')_2**2)) )

        # line1_in_2 = (F @ pts1.transpose(dim0=-2, dim1=-1)).transpose(dim0=-2, dim1=-1)
        # line2_in_1 = (F.transpose(dim0=-2, dim1=-1) @ pts2.transpose(dim0=-2, dim1=-1)).transpose(dim0=-2, dim1=-1)

        # Instead we can just transpose F once and switch the order of multiplication
        #F_t = Fm.transpose()
        line1_in_2 = np.matmul(Fm, pts1.transpose()).transpose()
        line2_in_1 = np.matmul(Fm.transpose(), pts2.transpose()).transpose()

        # numerator = (x'^T F x) ** 2
        numerator = (pts2 * line1_in_2).sum(axis=-1) ** 2
        #print(numerator.shape)
        # denominator = (((Fx)_1**2) + (Fx)_2**2)) +  (((F^Tx')_1**2) + (F^Tx')_2**2))
        #denominator = line1_in_2[..., :2].norm(2, axis=-1).pow(2) + line2_in_1[..., :2].norm(2, axis=-1).pow(2)
        denominator = np.linalg.norm(line1_in_2[..., :2], axis = -1) ** 2 + np.linalg.norm(line2_in_1[..., :2], axis = -1) ** 2
        #print(denominator.shape)
        out = numerator / denominator
        #print(out.shape)
        if squared:
            return np.mean(out)
        return (out + eps).sqrt()



    def symmetrical_epipolar_distance(self, pts1, pts2, camera_ids, Fm, squared = True, eps= 1e-8):
        """Return symmetrical epipolar distance for correspondences given the fundamental matrix.

        Args:
        pts1: correspondences from the left images with shape :math:`(*, N, (2|3))`. If they are not homogeneous,
                converted automatically.
        pts2: correspondences from the right images with shape :math:`(*, N, (2|3))`. If they are not homogeneous,
                converted automatically.
        Fm: Fundamental matrices with shape :math:`(*, 3, 3)`. Called Fm to avoid ambiguity with torch.nn.functional.
        squared: if True (default), the squared distance is returned.
        eps: Small constant for safe sqrt.

        Returns:
            the computed Symmetrical distance with shape :math:`(*, N)`.
        """

        pts1 = np.array(self.undistort(pts1, camera_ids[0]))
        pts2 = np.array(self.undistort(pts2, camera_ids[1]))
        
        if pts1.shape[-1] == 2:
            pts1 = self.convert_points_to_homogeneous(pts1)

        if pts2.shape[-1] == 2:
            pts2 = self.convert_points_to_homogeneous(pts2)

        # From Hartley and Zisserman, symmetric epipolar distance (11.10)
        # sed = (x'^T F x) ** 2 /  (((Fx)_1**2) + (Fx)_2**2)) +  1/ (((F^Tx')_1**2) + (F^Tx')_2**2))

        # line1_in_2 = (F @ pts1.transpose(dim0=-2, dim1=-1)).transpose(dim0=-2, dim1=-1)
        # line2_in_1 = (F.transpose(dim0=-2, dim1=-1) @ pts2.transpose(dim0=-2, dim1=-1)).transpose(dim0=-2, dim1=-1)

        # Instead we can just transpose F once and switch the order of multiplication
        F_t = Fm.transpose()
        line1_in_2 = np.matmul(pts1, F_t)
        line2_in_1 = np.matmul(pts2, Fm)

        # numerator = (x'^T F x) ** 2
        numerator = (pts2 * line1_in_2).sum(axis=-1) ** 2

        # denominator_inv =  1/ (((Fx)_1**2) + (Fx)_2**2)) +  1/ (((F^Tx')_1**2) + (F^Tx')_2**2))
        denominator_inv = 1.0 / (np.linalg.norm(line1_in_2[..., :2], axis=-1) ** 2) + 1.0 / (np.linalg.norm(line2_in_1[..., :2], axis=-1) ** 2)
        out = numerator * denominator_inv
        if squared:
            return np.mean(out)
        return (out + eps).sqrt()
    
    def get_Epipolar_Lines(self, points_2d_L, points_2d_R, camera_ids, F = None):
        """
        Get epipolar lines knowing the fundamental matrix after undistoring the points
        
        Parameters
        ----------
        F : numpy.ndarray
            3x3 fundamental matrix.
        x1, x2 : list
            List of (x,y) coordinate points on the image 1 (or image 2, respectively).
        """
        assert len(camera_ids) == 2, "Calculating Epipolar lines between 2 cameras only"
        
        if F is None:
            F = self.get_fundamental_matrix(camera_ids)
        
        points_2d_L = np.asarray(points_2d_L)
        upoints_2d_L = np.zeros_like(points_2d_L)
        for i in range(len(points_2d_L)):
            upoints_2d_L[i] = self.undistort(points_2d_L[i], camera_ids[0]).reshape(-1)
        
        points_2d_R = np.asarray(points_2d_R)
        upoints_2d_R = np.zeros_like(points_2d_R)
        for i in range(len(points_2d_R)):
            upoints_2d_R[i] = self.undistort(points_2d_R[i], camera_ids[1]).reshape(-1)

        epipolar_lines_on_R = []
        
        # Compute lines corresponding to upoints_2d_L points
        for ind in range(len(upoints_2d_L)):
            p = np.array([ [upoints_2d_L[ind][0]], [upoints_2d_L[ind][1]], [1]])
            epipolar_lines_on_R.append(F.dot(p)) # epipolar line on img2 (homogeneous coordinates)
        
        epipolar_lines_on_L = []
        
        # Compute lines corresponding to upoints_2d_R points
        for ind in range(len(upoints_2d_R)):
            p = np.array([ [upoints_2d_R[ind][0]], [upoints_2d_R[ind][1]], [1]])
            epipolar_lines_on_L.append(np.transpose(F).dot(p))   # epipolar line on img1 (homogeneous coordinates)
       
        return np.array(epipolar_lines_on_L), np.array(epipolar_lines_on_R)
    
    def get_essential_matrix(self, camera_ids):
        """ 
        camera_ids: camera id for each point comes from
        """
        F = self.get_fundamental_matrix(camera_ids)
        K1 = self.cameras[camera_ids[0]].K.copy()
        K2 = self.cameras[camera_ids[1]].K.copy()
        E = np.transpose(K2).dot(F).dot(K1)
        return E
    
    
   
    @classmethod
    def from_json(cls, filename, camera_ids=None):
        """Returns a Calibration intialized from a json file

        Args:
            filename (str): A json file containing calibration info.
        """
        with open(filename, "r") as f:
            data = json.loads(f.read())

        cameras = data["cameras"]
        return Calibration(cameras=cameras, selected_camera_ids=camera_ids)

    def save(self, filename):
        def listify(d):
            """Converts all ndarrays in dict d to lists"""
            if isinstance(d, np.ndarray):
                return d.tolist()
            if not isinstance(d, dict):
                return d
            return {k: listify(v) for k, v in d.items()}

        def jsonify(d):
            return json.dumps({str(id_): x for id_, x in listify(d).items()})

        data = {
            self.BOUNDS_KEY: self.world_ltrb,
            "cameras": {
                id_: camera.to_dict(legacy_format=False)
                for id_, camera in self.cameras.items()
            },
        }
        with open(filename, "w") as f:
            f.write(jsonify(data))
