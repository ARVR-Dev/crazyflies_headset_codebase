# import numpy as np


# class CollisionAvoidance:
#     def __init__(self, ellipsoid_radii, max_speed, sidestep_threshold):
#         """
#         初始化碰撞避免参数。
#         :param ellipsoid_radii: np.array, [x, y, z] 半径，用于定义 Voronoi 单元的形状。
#         :param max_speed: float, 最大速度。
#         :param sidestep_threshold: float, 触发 sidestep 的距离阈值。
#         """
#         self.ellipsoid_radii = ellipsoid_radii
#         self.max_speed = max_speed
#         self.sidestep_threshold = sidestep_threshold

#     def compute_voronoi_constraints(self, our_position, other_positions):
#         """
#         构建 Voronoi 单元的线性约束 Ax <= b。
#         :param our_position: np.array, 我们的当前位置。
#         :param other_positions: list of np.array, 其他无人机的位置。
#         :return: A, b 分别为线性不等式的左边矩阵和右边向量。
#         """
#         n_others = len(other_positions)
#         A = []
#         b = []

#         for other in other_positions:
#             delta = other - our_position
#             delta_stretched = delta / self.ellipsoid_radii
#             distance = np.linalg.norm(delta_stretched)
#             a = delta_stretched / distance
#             a /= np.linalg.norm(a)
#             b_value = distance / 2 - 1
#             A.append(a)
#             b.append(b_value)

#         return np.array(A), np.array(b)

#     def project_to_voronoi(self, goal, A, b):
#         """
#         将目标点投影到 Voronoi 单元中。
#         :param goal: np.array, 目标点。
#         :param A: np.array, 线性约束的左边矩阵。
#         :param b: np.array, 线性约束的右边向量。
#         :return: np.array, 投影后的点。
#         """
#         for _ in range(100):  # 最多迭代 100 次
#             for i in range(len(b)):
#                 if np.dot(A[i], goal) > b[i]:
#                     # 修正目标点，使其满足 Ax <= b
#                     goal -= (np.dot(A[i], goal) - b[i]) * A[i]

#         return goal

#     def avoid_collision(self, our_position, goal, other_positions):
#         """
#         碰撞避免逻辑。
#         :param our_position: np.array, 我们的当前位置。
#         :param goal: np.array, 我们的目标位置。
#         :param other_positions: list of np.array, 其他无人机的位置。
#         :return: np.array, 碰撞避免后的目标位置。
#         """
#         A, b = self.compute_voronoi_constraints(our_position, other_positions)
#         projected_goal = self.project_to_voronoi(goal, A, b)
#         distance = np.linalg.norm(projected_goal)

#         # 如果目标距离墙壁过近，触发 sidestep
#         if distance < self.sidestep_threshold:
#             sidestep_dir = np.cross(projected_goal, np.array([0, 0, 1]))
#             sidestep_dir /= np.linalg.norm(sidestep_dir)
#             sidestep_amount = (1.0 - distance / self.sidestep_threshold) ** 2
#             projected_goal += sidestep_dir * sidestep_amount

#         # 确保速度不超过限制
#         projected_goal = np.clip(projected_goal, -self.max_speed, self.max_speed)

#         return projected_goal

import numpy as np


class CollisionAvoidance:
    def __init__(self, ellipsoid_radii, max_speed, sidestep_threshold, bbox_min, bbox_max):
        """
        初始化碰撞避免参数。
        :param ellipsoid_radii: np.array, [x, y, z] 半径，用于定义 Voronoi 单元的形状。
        :param max_speed: float, 最大速度。
        :param sidestep_threshold: float, 触发 sidestep 的距离阈值。
        :param bbox_min: np.array, 飞行区域的最小边界。
        :param bbox_max: np.array, 飞行区域的最大边界。
        """
        self.ellipsoid_radii = ellipsoid_radii
        self.max_speed = max_speed
        self.sidestep_threshold = sidestep_threshold
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max

    def compute_voronoi_constraints(self, our_position, other_positions):
        """
        构建 Voronoi 单元的线性约束 Ax <= b。
        :param our_position: np.array, 我们的当前位置。
        :param other_positions: list of np.array, 其他无人机的位置。
        :return: A, b 分别为线性不等式的左边矩阵和右边向量。
        """
        n_others = len(other_positions)
        A = []
        b = []

        # 使用 ellipsoid_radii 的倒数进行伸缩变换
        radii_inv = 1.0 / self.ellipsoid_radii

        # # 固定距离阈值 = 1.5 (仅示例,小于这个距离才考虑开启colav)
        # distance_threshold = 1.5

        for other in other_positions:
        # # 先判断欧几里得距离（未伸缩）
        #     dist_real = np.linalg.norm(other - our_position)
        #     if dist_real > distance_threshold:
        #         # 若邻居太远，就不纳入约束
        #         continue


            delta = other - our_position
            delta_stretched = delta * radii_inv  # 进行坐标轴方向上的伸缩
            distance = np.linalg.norm(delta_stretched)
            if distance == 0:
                continue  # 跳过重叠点
            a = delta_stretched / distance  # 单位向量
            b_value = distance / 2 - 0.2
            A.append(a)
            b.append(b_value)

        # 添加全局边界框约束
        for dim in range(3):
            a = np.zeros(3)
            a[dim] = 1.0
            A.append(a)
            b.append(self.bbox_max[dim] - our_position[dim])

            a = np.zeros(3)
            a[dim] = -1.0
            A.append(a)
            b.append(our_position[dim] - self.bbox_min[dim])

        return np.array(A), np.array(b)

    def project_to_voronoi(self, goal, A, b, alpha=0.8, eps=1e-5):
        """
        将目标点投影到 Voronoi 单元中（带步长系数alpha、容忍度eps）。

        :param goal: np.array, 相对目标点 (一般是 goal - our_position).
        :param A: np.array, 线性约束的左边矩阵.
        :param b: np.array, 线性约束的右边向量.
        :param alpha: float, 步长系数(0<alpha<=1)，默认为0.5更平滑
        :param eps: float, 小于该阈值的violation视为可忽略(防止浮点抖动)
        :return: np.array, 投影后的点(若收敛)，或 None(若100次迭代仍未收敛).
        """
        max_iters = 100
        for iteration in range(max_iters):
            converged = True

            # 顺序检查各条约束
            for i in range(len(b)):
                violation = np.dot(A[i], goal) - b[i]

                # 若 violation > eps，则说明当前点违反了第i条约束
                if violation > eps:
                    # 根据步长alpha进行"部分"修正，而非一次全改
                    goal -= alpha * violation * A[i]
                    converged = False

            # 若一整轮没有任何约束被违反，则说明已经在可行域内
            if converged:
                break
        else:
            # 如果迭代max_iters次仍没收敛，视为投影失败
            return None

        return goal

    def avoid_collision(self, our_position, goal, other_positions):
        """
        碰撞避免逻辑。
        :param our_position: np.array, 我们的当前位置。
        :param goal: np.array, 我们的目标位置。
        :param other_positions: list of np.array, 其他无人机的位置。
        :return: np.array, 碰撞避免后的目标位置。
        """
        # 构建 Voronoi 约束
        A, b = self.compute_voronoi_constraints(our_position, other_positions)

        # 将目标点投影到 Voronoi 单元中
        projected_goal = self.project_to_voronoi(goal - our_position, A, b)
        if projected_goal is None:
            # 如果投影失败，返回当前位置作为目标点
            return our_position

        # 恢复到全局坐标系
        projected_goal += our_position

        # 3) 计算当前位置到边界的最小距离
        # -------------------------------
        distance_to_min = our_position - self.bbox_min
        distance_to_max = self.bbox_max - our_position
        distance_from_wall = np.min(np.concatenate([distance_to_min, distance_to_max]))

        # -------------------------------



        # # 检查距离，如果过近则触发 sidestep
        # distance_from_wall = np.linalg.norm(projected_goal - our_position)


        if distance_from_wall < self.sidestep_threshold:
            # 先计算侧步方向
            sidestep_dir = np.cross(projected_goal - our_position, np.array([0, 0, 1]))
            norm_sd = np.linalg.norm(sidestep_dir)
            if norm_sd > 1e-9:
                sidestep_dir /= norm_sd
                sidestep_amount = (1.0 - distance_from_wall / self.sidestep_threshold) ** 2

                # 做侧步
                projected_goal += sidestep_amount * sidestep_dir

                # **核心改动：侧步后再次投影**（相对坐标再投一次）
                # 这样可防止 sidestep 又违反了其他约束
                new_rel = self.project_to_voronoi(projected_goal - our_position, A, b)
                if new_rel is None:
                    # 如果侧步后依旧无法投影，则返回当前位置(或保持原结果)
                    return our_position
                projected_goal = new_rel + our_position

        # 限制速度
        velocity = projected_goal - our_position
        speed = np.linalg.norm(velocity)
        if speed > self.max_speed:
            velocity *= self.max_speed / speed
            projected_goal = our_position + velocity

        # 限制在边界框内
        projected_goal = np.clip(projected_goal, self.bbox_min, self.bbox_max)

        return projected_goal
