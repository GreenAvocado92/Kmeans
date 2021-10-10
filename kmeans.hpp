#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <iostream>
#include <vector>
// #include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <limits>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/io.h>
#include <glog/logging.h>
#include <limits>
#include <CGAL/squared_distance_3.h>
#include <CGAL/centroid.h>


class Kmeans
{
	typedef CGAL::Simple_cartesian<double> K;
	typedef K::Point_3 Point_3;
    typedef CGAL::Point_set_3<Point_3> Point_set;

public:
	Kmeans();
	~Kmeans() {}
	Kmeans(std::string file);
	/**
	 * @brief Set the Input object
	 * 
	 * @param source_path 
	 */
	bool setInput(std::string source);

	void setSimilarityModel(int flag);
	
	/**
	 * @brief Set the Numberof Clusters and select init centers indices
	 * 
	 * @param number_of_clusters the number of clusters
	 */
	void setNumberofClusters(int number_of_clusters);
	/**
	 * @brief Set the Init Cluster Center object
	 * 
	 * @param init_centers the indices of init_centers point, if you should assign the init point,
	 * 					   you should use this function
	 */
	void setInitClusterCenter(const std::vector<Point_3> init_centers);

	/**
	 * @brief Set the Max Iterations
	 * 
	 * @param maxiter the max number of iteration
	 */
	void setMaxIterations(int maxiter) {
		maxiter_ = maxiter;
	};

	/**
	 * @brief Set the Max Correspondence Distance 
	 * 
	 * @param maxcor 
	 */
	void setMaxCorrespondenceDistance(float maxcor) {
		maxcor_ = maxcor;
	};

	/**
	 * @brief execute kmean
	 * 
	 * @return true 
	 * @return false 
	 */
	bool compute();

	/**
	 * @brief get result
	 * 
	 */
	bool save_clusters(std::string path, int minsize);

private:
	/**
	 * @brief input point cloud
	 */
	Point_set points_;

	/**
	 * @brief the indices of point which is center
	 * 
	 */
	// std::vector<int> centers_indices_;
	std::vector<Point_3> centers_points_;

	/**
	 * @brief indices of each clusters
	 * 
	 */
	std::vector<std::vector<int>> clusters_indices_;

	/**
	 * @brief assign centers
	 * 
	 */
	void assign_centers();

	/**
	 * @brief compute threshold between centers and cluster center
	 * 
	 * @return true meet the requirement
	 * @return false 
	 */
	bool compute_threshold();

	/**
	 * @brief some parameters
	 * 
	 */
	int maxiter_;
	int finaliter_;
	float maxcor_;
	int number_of_clusters_;
};

Kmeans::Kmeans()
{
	LOG(INFO) << "construct kmeans ...";
	finaliter_ = 0;
	maxiter_ = 100;
	maxcor_ = 10.0;
	number_of_clusters_ = 10;
	clusters_indices_.resize(number_of_clusters_);
}

Kmeans::Kmeans(std::string file)
{
	LOG(INFO) << "construct kmeans ...";
	setInput(file);

	finaliter_ = 0;
	maxiter_ = 100;
	maxcor_ = 1.0;
	number_of_clusters_ = 10;
	clusters_indices_.resize(number_of_clusters_);
}

bool Kmeans::setInput(std::string source)
{
	LOG(INFO) << "read data from file path";
	CGAL::IO::read_XYZ(source, points_);
	if (points_.size() == 0) {
		LOG(INFO) << "input data is zero";
		return false;
	}
	return true;
}

void Kmeans::setNumberofClusters(int number_of_clusters)
{
	LOG(INFO) << "set number of clusters";
	number_of_clusters_ = number_of_clusters;
    centers_points_.resize(number_of_clusters_);

	srand((int)time(0));
	// selsect number_of_clusters different points
	for (int i = 0; i < number_of_clusters_; i++) {
		centers_points_[i] = points_.point(rand() % points_.size());
	}
}

void Kmeans::setInitClusterCenter(const std::vector<Point_3> init_centers)
{
	number_of_clusters_ = init_centers.size();
    centers_points_.resize(number_of_clusters_);
	for (int i = 0; i < init_centers.size(); ++i)
		centers_points_[i] = init_centers[i];
}

bool Kmeans::compute_threshold() {
	// get points from points_
	LOG(INFO) << "computer center and updata centers";
	bool isUpdate = false;
	float x = 0, y = 0, z = 0;
	// Point_set temp_points;
	std::vector<Point_3> temp_points;

	for (int i = 0; i < clusters_indices_.size(); i++)  {
		Point_3 center = centers_points_[i];
		for (int j = 0; j < clusters_indices_[i].size(); j++) {
			Point_3 p = points_.point(j);
			temp_points.push_back(p);
		}
		Point_3 temp_centroid = CGAL::centroid(temp_points.begin(),
											   temp_points.end(),
											   CGAL::Dimension_tag<0>());

		float temp = CGAL::squared_distance(center, temp_centroid);
		if (temp > maxcor_) {
			centers_points_[i] = Point_3(x, y, z);
			isUpdate = true;
		}
	}
	LOG(INFO) << "compute_threshold && update center successful";
	return isUpdate;
}

void Kmeans::assign_centers() {
	LOG(INFO) << "assign points to target centers";
	for (int i = 0; i < points_.size(); ++i) {
		float min_distance = std::numeric_limits<float>::max();
		int min_dis_clusters = -1;
		Point_3 temp_p = points_.point(i);
		for (int j = 0; j < number_of_clusters_; ++j) {
			float temp = CGAL::squared_distance(temp_p, centers_points_[j]);
			if (temp < min_distance) {
				min_dis_clusters = j;
				min_distance = temp;
			}
		}
		clusters_indices_[min_dis_clusters].push_back(i);
	}
	LOG(INFO) << "assign_centers successful";
}

bool Kmeans::compute() {
	LOG(INFO) << "compute ... ";
	bool isUpdate = true;
	int iter = 0;
	while (isUpdate) {
		assign_centers();
		isUpdate = compute_threshold();
		iter ++;
		if (iter == maxiter_)
			break;
	}
	finaliter_ = iter;
    LOG(INFO) << "compute successful ";

	return true;
}

// @TODOS add
bool Kmeans::save_clusters(std::string path, int minsize = std::numeric_limits<unsigned int>::min()) {
    std::string output_file = "";
    for (int i = 0; i < clusters_indices_.size(); ++i) {
        output_file = path + "/" + std::to_string(i) + ".asc";
        LOG(INFO) << "output_file = " << output_file;
        auto temp = clusters_indices_[i];
        if (temp.size() > minsize) {
            std::ofstream out(output_file);
            for (int j = 0; j < temp.size(); j++)
                out << points_.point(temp[j]).x() << " " 
					<< points_.point(temp[j]).y() << " "
					<< points_.point(temp[j]).z() << std::endl;
            out.close();
        }        
    }

    return true;
}


#endif
