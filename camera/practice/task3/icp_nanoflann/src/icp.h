#ifndef ICP_H
#define ICP_H

#include "nanoflann.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
    nn_kd_tree;

class ICP {
 public:
    explicit ICP(int m_it);
    Eigen::MatrixXd best_fit_transform(const Eigen::MatrixXd &pointcloud_A,
                                       const Eigen::MatrixXd &pointcloud_B);
    float euc_dist(const Eigen::Vector3d &pt_a, const Eigen::Vector3d &pt_b);
    void calc_closest_neighbors_kdtree(const Eigen::MatrixXd &src, const nn_kd_tree &dst_tree);
    void run_scan_matcher(const Eigen::MatrixXd &pointcloud_A, const Eigen::MatrixXd &pointcloud_B,
                          double tolerance = 0.00001);

    // Extract final ICP results
    Eigen::Matrix4d get_transformation() { return transform_matr_; }
    std::vector<double> get_last_distances() { return dists_; }
    int get_last_iterations() { return iters_; }
    Eigen::MatrixXd get_transformed_pointcloud() { return src_3d_; }

 private:
    int max_iters_{0};
    int iters_{0};
    Eigen::Matrix4d transform_matr_;
    std::vector<int> indices_;
    std::vector<double> dists_;
    Eigen::MatrixXd src_3d_;
};

#endif  // ICP_H
